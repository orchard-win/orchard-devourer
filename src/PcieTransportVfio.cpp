#include "PcieTransport.h"

/* Linux host backend: vfio-pci. Compiled when DEVOURER_PCIE=ON on Linux. */

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <linux/vfio.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace devourer {

namespace {
constexpr uint16_t kPciDidRtl8852ce = 0xC852;
} /* namespace */

struct PcieTransport::Os {
  int container = -1, group = -1, device = -1;
  int msi_evt = -1; /* eventfd signalled per MSI; -1 = polling mode */
  volatile uint8_t *mmio = nullptr;
  uint64_t cfg_region_off = 0;
  size_t cfg_region_len = 0;
  uint8_t *slab = nullptr; /* DMA slab VA (anonymous, VFIO-pinned) */
  size_t slab_len = 0;
};

PcieTransport::PcieTransport(Logger_t logger, Config cfg)
    : _logger(std::move(logger)), _cfg(cfg), _os(std::make_unique<Os>()) {}

int PcieTransport::msi_fd() const { return _os->msi_evt; }

bool PcieTransport::setup_msi() {
  struct vfio_irq_info info{};
  info.argsz = sizeof(info);
  info.index = VFIO_PCI_MSI_IRQ_INDEX;
  if (ioctl(_os->device, VFIO_DEVICE_GET_IRQ_INFO, &info) < 0 ||
      info.count < 1) {
    _logger->warn("PcieTransport: no MSI IRQ available");
    return false;
  }
  int evt = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (evt < 0)
    return false;
  /* One MSI vector -> the eventfd. */
  char buf[sizeof(struct vfio_irq_set) + sizeof(int32_t)] = {};
  auto *is = reinterpret_cast<struct vfio_irq_set *>(buf);
  is->argsz = sizeof(buf);
  is->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
  is->index = VFIO_PCI_MSI_IRQ_INDEX;
  is->start = 0;
  is->count = 1;
  memcpy(is->data, &evt, sizeof(int32_t));
  if (ioctl(_os->device, VFIO_DEVICE_SET_IRQS, is) < 0) {
    _logger->warn("PcieTransport: VFIO_DEVICE_SET_IRQS(MSI) failed: {}",
                  strerror(errno));
    close(evt);
    return false;
  }
  _os->msi_evt = evt;
  return true;
}

PcieTransport::~PcieTransport() {
  if (_os->msi_evt >= 0) {
    if (_os->mmio && _dma)
      _dma->irq_mask(); /* mask before dropping the vector */
    struct vfio_irq_set off{};
    off.argsz = sizeof(off);
    off.flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER;
    off.index = VFIO_PCI_MSI_IRQ_INDEX;
    off.count = 0;
    ioctl(_os->device, VFIO_DEVICE_SET_IRQS, &off);
    close(_os->msi_evt);
  }
  _dma.reset();
  if (_os->mmio)
    munmap(const_cast<uint8_t *>(_os->mmio), _mmio_len);
  if (_os->slab && _os->container >= 0) {
    struct vfio_iommu_type1_dma_unmap um{};
    um.argsz = sizeof(um);
    um.iova = _cfg.iova_base;
    um.size = _os->slab_len;
    ioctl(_os->container, VFIO_IOMMU_UNMAP_DMA, &um);
  }
  if (_os->slab)
    munmap(_os->slab, _os->slab_len);
  if (_os->device >= 0)
    close(_os->device);
  if (_os->group >= 0)
    close(_os->group);
  if (_os->container >= 0)
    close(_os->container);
}

bool PcieTransport::open_device(const std::string &bdf) {
  /* IOMMU group number from sysfs. */
  std::string link = "/sys/bus/pci/devices/" + bdf + "/iommu_group";
  char buf[256];
  ssize_t n = readlink(link.c_str(), buf, sizeof(buf) - 1);
  if (n <= 0) {
    _logger->error("PcieTransport: readlink({}) failed: {}", link,
                   strerror(errno));
    return false;
  }
  buf[n] = 0;
  const char *slash = strrchr(buf, '/');
  std::string group_num = slash ? slash + 1 : buf;

  _os->container = open("/dev/vfio/vfio", O_RDWR);
  if (_os->container < 0) {
    _logger->error("PcieTransport: open /dev/vfio/vfio failed: {} (modprobe "
                   "vfio-pci?)",
                   strerror(errno));
    return false;
  }
  if (ioctl(_os->container, VFIO_GET_API_VERSION) != VFIO_API_VERSION) {
    _logger->error("PcieTransport: VFIO API version mismatch");
    return false;
  }
  int iommu_type = 0;
  if (ioctl(_os->container, VFIO_CHECK_EXTENSION, VFIO_TYPE1v2_IOMMU) == 1)
    iommu_type = VFIO_TYPE1v2_IOMMU;
  else if (ioctl(_os->container, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) == 1)
    iommu_type = VFIO_TYPE1_IOMMU;
  else {
    _logger->error("PcieTransport: no Type1 IOMMU support");
    return false;
  }

  std::string group_path = "/dev/vfio/" + group_num;
  _os->group = open(group_path.c_str(), O_RDWR);
  if (_os->group < 0) {
    _logger->error("PcieTransport: open {} failed: {} (device bound to "
                   "vfio-pci? permissions?)",
                   group_path, strerror(errno));
    return false;
  }
  struct vfio_group_status st{};
  st.argsz = sizeof(st);
  if (ioctl(_os->group, VFIO_GROUP_GET_STATUS, &st) < 0 ||
      !(st.flags & VFIO_GROUP_FLAGS_VIABLE)) {
    _logger->error("PcieTransport: IOMMU group {} not viable (all devices in "
                   "the group must be bound to vfio-pci)",
                   group_num);
    return false;
  }
  if (ioctl(_os->group, VFIO_GROUP_SET_CONTAINER, &_os->container) < 0) {
    _logger->error("PcieTransport: GROUP_SET_CONTAINER failed: {}",
                   strerror(errno));
    return false;
  }
  if (ioctl(_os->container, VFIO_SET_IOMMU, iommu_type) < 0) {
    _logger->error("PcieTransport: SET_IOMMU failed: {}", strerror(errno));
    return false;
  }
  _os->device = ioctl(_os->group, VFIO_GROUP_GET_DEVICE_FD, bdf.c_str());
  if (_os->device < 0) {
    _logger->error("PcieTransport: GET_DEVICE_FD({}) failed: {}", bdf,
                   strerror(errno));
    return false;
  }
  _logger->info("PcieTransport: vfio group {} opened for {}", group_num, bdf);

  struct vfio_region_info reg{};
  reg.argsz = sizeof(reg);
  reg.index = VFIO_PCI_BAR2_REGION_INDEX;
  if (ioctl(_os->device, VFIO_DEVICE_GET_REGION_INFO, &reg) < 0) {
    _logger->error("PcieTransport: BAR2 region info failed: {}",
                   strerror(errno));
    return false;
  }
  if (!(reg.flags & VFIO_REGION_INFO_FLAG_MMAP) || reg.size == 0) {
    _logger->error("PcieTransport: BAR2 not mmap-able (size={} flags={:#x})",
                   (unsigned long long)reg.size, reg.flags);
    return false;
  }
  void *p = mmap(nullptr, reg.size, PROT_READ | PROT_WRITE, MAP_SHARED,
                 _os->device, reg.offset);
  if (p == MAP_FAILED) {
    _logger->error("PcieTransport: BAR2 mmap failed: {}", strerror(errno));
    return false;
  }
  _os->mmio = static_cast<volatile uint8_t *>(p);
  _mmio_len = reg.size;
  return true;
}

uint32_t PcieTransport::reg_read(uint32_t off, size_t width) {
  volatile uint8_t *p = _os->mmio + off;
  if (width == 4)
    return *reinterpret_cast<volatile uint32_t *>(p);
  if (width == 2)
    return *reinterpret_cast<volatile uint16_t *>(p);
  return *p;
}

void PcieTransport::reg_write(uint32_t off, size_t width, uint32_t v) {
  volatile uint8_t *p = _os->mmio + off;
  if (width == 4)
    *reinterpret_cast<volatile uint32_t *>(p) = v;
  else if (width == 2)
    *reinterpret_cast<volatile uint16_t *>(p) = static_cast<uint16_t>(v);
  else
    *p = static_cast<uint8_t>(v);
}

bool PcieTransport::cfg_read(uint32_t off, void *buf, size_t len) {
  return pread(_os->device, buf, len, _os->cfg_region_off + off) ==
         static_cast<ssize_t>(len);
}

bool PcieTransport::cfg_write(uint32_t off, const void *buf, size_t len) {
  return pwrite(_os->device, buf, len, _os->cfg_region_off + off) ==
         static_cast<ssize_t>(len);
}

bool PcieTransport::setup_config_space() {
  struct vfio_region_info reg{};
  reg.argsz = sizeof(reg);
  reg.index = VFIO_PCI_CONFIG_REGION_INDEX;
  if (ioctl(_os->device, VFIO_DEVICE_GET_REGION_INFO, &reg) < 0) {
    _logger->error("PcieTransport: config region info failed: {}",
                   strerror(errno));
    return false;
  }
  _os->cfg_region_off = reg.offset;
  _os->cfg_region_len = reg.size;

  uint16_t vid = 0, did = 0;
  cfg_read(0x00, &vid, 2);
  cfg_read(0x02, &did, 2);
  if (vid == 0xFFFF) {
    _logger->error("PcieTransport: config space reads 0xFFFF — link down?");
    return false;
  }
  _pci_vid = vid;
  _pci_did = did;
  _logger->info("PcieTransport: PCI device {:04x}:{:04x}", vid, did);

  /* Memory + bus-master enable. Without bus-master every register read works
   * but no DMA moves — the classic silent-failure trap. */
  uint16_t cmd = 0;
  cfg_read(0x04, &cmd, 2);
  cmd |= 0x2 /* MEMORY */ | 0x4 /* MASTER */;
  cfg_write(0x04, &cmd, 2);

  /* Find the PCI Express capability (id 0x10) for LNKCTL / DEVCTL2. */
  uint8_t cap_ptr = 0;
  cfg_read(0x34, &cap_ptr, 1);
  uint32_t pcie_cap = 0;
  for (int guard = 0; cap_ptr && guard < 48; guard++) {
    uint8_t id = 0, next = 0;
    cfg_read(cap_ptr, &id, 1);
    cfg_read(cap_ptr + 1, &next, 1);
    if (id == 0x10) {
      pcie_cap = cap_ptr;
      break;
    }
    cap_ptr = next;
  }
  _pcie_cap = pcie_cap;
  if (pcie_cap) {
    /* Clear ASPM (LNKCTL[1:0]) during bring-up — L1 entry on a half-configured
     * link is a known hang source. */
    uint16_t lnkctl = 0;
    cfg_read(pcie_cap + 0x10, &lnkctl, 2);
    if (lnkctl & 0x3) {
      lnkctl &= ~0x3;
      cfg_write(pcie_cap + 0x10, &lnkctl, 2);
      _logger->info("PcieTransport: ASPM disabled for bring-up");
    }
    /* Disable completion timeout (DEVCTL2 bit4) — rtw88 does this specifically
     * for the 8821C (rtw_pci_phy_cfg). Only there: with the timeout off, a
     * read of a device that stops answering never completes and the CPU
     * hangs in the MMIO load instead of seeing all-ones, so the AX parts
     * (rtw89 leaves the timeout alone) keep it. */
    if (_pci_did != kPciDidRtl8852ce) {
      uint16_t devctl2 = 0;
      cfg_read(pcie_cap + 0x28, &devctl2, 2);
      devctl2 |= 1u << 4;
      cfg_write(pcie_cap + 0x28, &devctl2, 2);
    }
  } else {
    _logger->warn("PcieTransport: PCIe capability not found — skipping "
                  "ASPM/completion-timeout config");
  }
  return true;
}

bool PcieTransport::init_dma() {
  _os->slab_len = _dma->slab_bytes();
  void *p = mmap(nullptr, _os->slab_len, PROT_READ | PROT_WRITE,
                 MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  if (p == MAP_FAILED) {
    _logger->error("PcieTransport: DMA slab mmap({} KiB) failed: {}",
                   _os->slab_len / 1024, strerror(errno));
    return false;
  }
  _os->slab = static_cast<uint8_t *>(p);
  memset(_os->slab, 0, _os->slab_len);

  if (_cfg.iova_base + _os->slab_len > (1ull << 32)) {
    _logger->error("PcieTransport: IOVA base 0x{:x} + slab exceeds 4 GiB (the "
                   "descriptor dma fields are 32-bit)",
                   _cfg.iova_base);
    return false;
  }
  struct vfio_iommu_type1_dma_map m{};
  m.argsz = sizeof(m);
  m.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;
  m.vaddr = reinterpret_cast<uint64_t>(_os->slab);
  m.iova = _cfg.iova_base;
  m.size = _os->slab_len;
  if (ioctl(_os->container, VFIO_IOMMU_MAP_DMA, &m) < 0) {
    _logger->error("PcieTransport: VFIO_IOMMU_MAP_DMA failed: {}",
                   strerror(errno));
    return false;
  }
  /* One IOVA run for the whole slab; closing the device fd resets the
   * function, so the plane's quiesce ops are not needed here. */
  _slab.va = _os->slab;
  _slab.len = _os->slab_len;
  _slab.chunk_bytes = 0;
  _slab.chunk_iova = {_cfg.iova_base};
  return _dma->attach(_slab);
}

} /* namespace devourer */
