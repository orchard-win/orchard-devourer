#include "PcieTransport.h"

/* Windows host backend. Registers go through the regproxy driver, DMA through
 * locked 2 MiB blocks whose physical addresses serve as bus addresses. */

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <cfgmgr32.h>
#include <setupapi.h>
#include <winioctl.h>

#include <algorithm>
#include <cstring>
#include <cwchar>
#include <vector>

#include "PhysicalBlocks.h"
#include "regproxy_ioctl.h"

namespace devourer {

namespace {

constexpr size_t kPageBytes = 4096;
constexpr size_t kBlockBytes = size_t{2} << 20;
constexpr size_t kPagesPerBlock = kBlockBytes / kPageBytes;
constexpr unsigned kPageShift = 12;
constexpr ULONG_PTR kFrameAt4GiB = ULONG_PTR{1} << (32 - kPageShift);
/* The system hands out blocks wherever it likes: ask for more than needed and
 * keep the lowest. The last attempt takes every free large page. */
constexpr size_t kOverRequest[] = {4, 16};
constexpr size_t kRequestEverything = 4096;
/* Single pages come from anywhere too, and few of them from below 4 GiB:
 * how many to ask for, in turn, when some must be that low. */
constexpr size_t kPageRequests[] = {size_t{1} << 15, size_t{1} << 17,
                                    size_t{1} << 19};

using AllocatePages2Fn = BOOL(WINAPI *)(HANDLE, PULONG_PTR, PULONG_PTR,
                                        PMEM_EXTENDED_PARAMETER, ULONG);

bool enable_lock_memory_privilege() {
  HANDLE token = nullptr;
  if (!OpenProcessToken(GetCurrentProcess(),
                        TOKEN_ADJUST_PRIVILEGES | TOKEN_QUERY, &token))
    return false;
  TOKEN_PRIVILEGES privileges{};
  privileges.PrivilegeCount = 1;
  privileges.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED;
  const bool found = LookupPrivilegeValueW(nullptr, L"SeLockMemoryPrivilege",
                                           &privileges.Privileges[0].Luid);
  if (found)
    AdjustTokenPrivileges(token, FALSE, &privileges, 0, nullptr, nullptr);
  /* AdjustTokenPrivileges succeeds even when the privilege is not held. */
  const bool enabled = found && GetLastError() == ERROR_SUCCESS;
  CloseHandle(token);
  return enabled;
}

/* Allocates `count` locked 2 MiB blocks; each entry of the result is the
 * frame number of a block's first 4 KiB page. May return fewer. */
std::vector<ULONG_PTR> allocate_blocks(size_t count) {
  /* Looked up at run time: the import library that carries it varies by SDK. */
  const auto allocate = reinterpret_cast<AllocatePages2Fn>(GetProcAddress(
      GetModuleHandleW(L"kernelbase.dll"), "AllocateUserPhysicalPages2"));
  std::vector<ULONG_PTR> frames(count);
  MEM_EXTENDED_PARAMETER large{};
  large.Type = MemExtendedParameterAttributeFlags;
  large.ULong64 = MEM_EXTENDED_PARAMETER_NONPAGED_LARGE;
  ULONG_PTR got = count;
  if (!allocate ||
      !allocate(GetCurrentProcess(), &got, frames.data(), &large, 1))
    got = 0;
  frames.resize(got);
  return frames;
}

/* Allocates up to `count` locked 4 KiB pages and returns their frames. */
std::vector<ULONG_PTR> allocate_pages(size_t count) {
  std::vector<ULONG_PTR> frames(count);
  ULONG_PTR got = count;
  if (!AllocateUserPhysicalPages(GetCurrentProcess(), &got, frames.data()))
    got = 0;
  frames.resize(got);
  return frames;
}

void free_blocks(std::vector<ULONG_PTR> &frames) {
  ULONG_PTR count = frames.size();
  if (count)
    FreeUserPhysicalPages(GetCurrentProcess(), &count, frames.data());
  frames.clear();
}

/* "PCI\VEN_10EC&DEV_C852&..." -> the hex field after `key`. */
uint16_t instance_id_field(const wchar_t *instance_id, const wchar_t *key) {
  const wchar_t *at = std::wcsstr(instance_id, key);
  return at ? static_cast<uint16_t>(
                  std::wcstoul(at + std::wcslen(key), nullptr, 16))
            : 0;
}

} /* namespace */

struct PcieTransport::Os {
  HANDLE proxy = INVALID_HANDLE_VALUE;
  uint8_t *slab = nullptr;          /* one MEM_PHYSICAL view over all of it */
  std::vector<ULONG_PTR> head;      /* frames of single pages mapped first */
  std::vector<ULONG_PTR> blocks;    /* first frame of each block mapped next */
  std::vector<REGPROXY_OP> quiesce; /* stops the plane's DMA engine */
  std::atomic<bool> io_error_logged{false};
};

PcieTransport::PcieTransport(Logger_t logger, Config cfg)
    : _logger(std::move(logger)), _cfg(cfg), _os(std::make_unique<Os>()) {}

PcieTransport::~PcieTransport() {
  _dma.reset();
  if (_os->proxy != INVALID_HANDLE_VALUE && !_os->quiesce.empty()) {
    /* The device object stops DMA on its own teardown; repeating it here is
     * what makes freeing the slab safe when that object never existed. */
    DWORD returned = 0;
    const DWORD bytes =
        static_cast<DWORD>(_os->quiesce.size() * sizeof(REGPROXY_OP));
    DeviceIoControl(_os->proxy, IOCTL_REGPROXY_BATCH, _os->quiesce.data(),
                    bytes, _os->quiesce.data(), bytes, &returned, nullptr);
    DeviceIoControl(_os->proxy, IOCTL_REGPROXY_SET_CLOSE_OPS, nullptr, 0,
                    nullptr, 0, &returned, nullptr);
  }
  if (_os->slab) {
    MapUserPhysicalPages(_os->slab,
                         _os->head.size() + _os->blocks.size() * kPagesPerBlock,
                         nullptr);
    VirtualFree(_os->slab, 0, MEM_RELEASE);
  }
  free_blocks(_os->head);
  free_blocks(_os->blocks);
  if (_os->proxy != INVALID_HANDLE_VALUE)
    CloseHandle(_os->proxy);
}

int PcieTransport::msi_fd() const { return -1; }

bool PcieTransport::setup_msi() { return false; }

bool PcieTransport::open_device(const std::string &bdf) {
  (void)bdf;
  const GUID *iface = &GUID_DEVINTERFACE_WINAWDL_REGPROXY;
  HDEVINFO set = SetupDiGetClassDevsW(iface, nullptr, nullptr,
                                      DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
  if (set == INVALID_HANDLE_VALUE) {
    _logger->error("PcieTransport: device enumeration failed ({})",
                   GetLastError());
    return false;
  }

  SP_DEVICE_INTERFACE_DATA data{};
  data.cbSize = sizeof(data);
  if (SetupDiEnumDeviceInterfaces(set, nullptr, iface, 0, &data)) {
    DWORD size = 0;
    SetupDiGetDeviceInterfaceDetailW(set, &data, nullptr, 0, &size, nullptr);
    std::vector<uint8_t> storage(size);
    auto *detail =
        reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W *>(storage.data());
    detail->cbSize = sizeof(*detail);
    SP_DEVINFO_DATA devinfo{};
    devinfo.cbSize = sizeof(devinfo);
    wchar_t instance_id[MAX_DEVICE_ID_LEN] = {};
    if (SetupDiGetDeviceInterfaceDetailW(set, &data, detail, size, nullptr,
                                         &devinfo) &&
        SetupDiGetDeviceInstanceIdW(set, &devinfo, instance_id,
                                    MAX_DEVICE_ID_LEN, nullptr)) {
      _pci_vid = instance_id_field(instance_id, L"VEN_");
      _pci_did = instance_id_field(instance_id, L"DEV_");
      _os->proxy =
          CreateFileW(detail->DevicePath, GENERIC_READ | GENERIC_WRITE, 0,
                      nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    }
  }
  const DWORD open_error = GetLastError();
  SetupDiDestroyDeviceInfoList(set);
  if (_os->proxy == INVALID_HANDLE_VALUE) {
    _logger->error("PcieTransport: register proxy not openable ({}): is the "
                   "driver bound to the card, the process elevated, and no "
                   "other client running?",
                   open_error);
    return false;
  }

  REGPROXY_INFO info{};
  DWORD returned = 0;
  if (!DeviceIoControl(_os->proxy, IOCTL_REGPROXY_GET_INFO, nullptr, 0, &info,
                       sizeof(info), &returned, nullptr)) {
    _logger->error("PcieTransport: register proxy GET_INFO failed ({})",
                   GetLastError());
    return false;
  }
  if (info.version != REGPROXY_VERSION) {
    _logger->error("PcieTransport: register proxy speaks version {}, this "
                   "build needs {} — reinstall the driver",
                   info.version, REGPROXY_VERSION);
    return false;
  }
  _mmio_len = info.bar_len;
  _logger->info("PcieTransport: register proxy opened, BAR2 {} KiB at 0x{:x}",
                _mmio_len / 1024, info.bar_phys);
  return true;
}

bool PcieTransport::setup_config_space() {
  if (_pci_vid == 0 || _pci_did == 0) {
    _logger->error(
        "PcieTransport: PCI ids not found in the device instance id");
    return false;
  }
  _logger->info(
      "PcieTransport: PCI device {:04x}:{:04x}; config space is out "
      "of reach here, bus mastering and ASPM stay as Windows set them",
      _pci_vid, _pci_did);
  return true;
}

bool PcieTransport::cfg_read(uint32_t, void *, size_t) { return false; }

bool PcieTransport::cfg_write(uint32_t, const void *, size_t) { return false; }

uint32_t PcieTransport::reg_read(uint32_t off, size_t width) {
  REGPROXY_OP op{};
  op.offset = off;
  op.width = static_cast<unsigned char>(width);
  op.kind = REGPROXY_READ;
  DWORD returned = 0;
  if (DeviceIoControl(_os->proxy, IOCTL_REGPROXY_BATCH, &op, sizeof(op), &op,
                      sizeof(op), &returned, nullptr))
    return op.value;
  if (!_os->io_error_logged.exchange(true))
    _logger->error("PcieTransport: register read 0x{:x} failed ({})", off,
                   GetLastError());
  /* What a device that stopped answering reads as; callers already cope. */
  return 0xFFFFFFFFu >> (8 * (4 - width));
}

void PcieTransport::reg_write(uint32_t off, size_t width, uint32_t v) {
  REGPROXY_OP op{};
  op.offset = off;
  op.value = v;
  op.width = static_cast<unsigned char>(width);
  op.kind = REGPROXY_WRITE;
  DWORD returned = 0;
  if (!DeviceIoControl(_os->proxy, IOCTL_REGPROXY_BATCH, &op, sizeof(op), &op,
                       sizeof(op), &returned, nullptr) &&
      !_os->io_error_logged.exchange(true))
    _logger->error("PcieTransport: register write 0x{:x} failed ({})", off,
                   GetLastError());
}

bool PcieTransport::init_dma() {
  const size_t need = (_dma->slab_bytes() + kBlockBytes - 1) / kBlockBytes;
  if (!enable_lock_memory_privilege()) {
    _logger->error("PcieTransport: SeLockMemoryPrivilege is not available — "
                   "run elevated");
    return false;
  }

  const unsigned address_bits = _dma->dma_address_bits();
  const ULONG_PTR limit_frame = ULONG_PTR{1} << (address_bits - kPageShift);
  const size_t low_need =
      (_dma->low_slab_bytes() + kBlockBytes - 1) / kBlockBytes;
  std::vector<size_t> requests;
  for (size_t factor : kOverRequest)
    requests.push_back(need * factor);
  requests.push_back(kRequestEverything);
  for (size_t request : requests) {
    std::vector<ULONG_PTR> frames = allocate_blocks(request);
    _os->blocks = take_lowest_blocks(frames, need, kPagesPerBlock, limit_frame,
                                     low_need, kFrameAt4GiB);
    free_blocks(frames);
    if (!_os->blocks.empty())
      break;
  }
  /* A low block can be missing where low pages are not: the part that must
   * be low is then made of those, and only the rest of blocks. */
  const size_t head_pages = low_need * kPagesPerBlock;
  if (_os->blocks.empty() && low_need) {
    for (size_t request : requests) {
      std::vector<ULONG_PTR> frames = allocate_blocks(request);
      _os->blocks = take_lowest_blocks(frames, need - low_need, kPagesPerBlock,
                                       limit_frame);
      free_blocks(frames);
      if (!_os->blocks.empty())
        break;
    }
    _logger->info("PcieTransport: no 2 MiB block is free below 4 GiB; {} of "
                  "the {} other blocks found",
                  _os->blocks.size(), need - low_need);
    for (size_t request : kPageRequests) {
      if (_os->blocks.empty() || !_os->head.empty())
        break;
      std::vector<ULONG_PTR> frames = allocate_pages(request);
      const auto low = std::count_if(
          frames.begin(), frames.end(),
          [](ULONG_PTR frame) { return frame < kFrameAt4GiB; });
      _logger->info("PcieTransport: of {} single pages, {} are below 4 GiB "
                    "({} wanted)",
                    frames.size(), low, head_pages);
      _os->head = take_lowest_blocks(frames, head_pages, 1, kFrameAt4GiB);
      free_blocks(frames);
    }
    if (_os->head.empty())
      free_blocks(_os->blocks);
  }
  if (_os->blocks.empty()) {
    _logger->error("PcieTransport: could not get {} locked 2 MiB blocks within "
                   "{} address bits with {} of them, or as many single pages, "
                   "below 4 GiB — free memory is too fragmented; try after a "
                   "reboot",
                   need, address_bits, low_need);
    return false;
  }

  const size_t slab_len = need * kBlockBytes;
  _os->slab = static_cast<uint8_t *>(VirtualAlloc(
      nullptr, slab_len, MEM_RESERVE | MEM_PHYSICAL, PAGE_READWRITE));
  if (!_os->slab) {
    _logger->error("PcieTransport: reserving the slab view failed ({})",
                   GetLastError());
    return false;
  }
  const size_t head_len = _os->head.size() * kPageBytes;
  if (!_os->head.empty() &&
      !MapUserPhysicalPages(_os->slab, _os->head.size(), _os->head.data())) {
    _logger->error("PcieTransport: mapping the slab's low pages failed ({})",
                   GetLastError());
    return false;
  }
  /* A block maps as its 512 consecutive 4 KiB frames; the documented large
   * page flags are refused for AWE views. */
  std::vector<ULONG_PTR> pages(kPagesPerBlock);
  for (size_t i = 0; i < _os->blocks.size(); i++) {
    for (size_t page = 0; page < kPagesPerBlock; page++)
      pages[page] = _os->blocks[i] + page;
    if (!MapUserPhysicalPages(_os->slab + head_len + i * kBlockBytes,
                              kPagesPerBlock, pages.data())) {
      _logger->error("PcieTransport: mapping slab block {} failed ({})", i,
                     GetLastError());
      return false;
    }
  }
  std::memset(_os->slab, 0, slab_len);

  _slab.va = _os->slab;
  _slab.len = slab_len;
  _slab.chunk_bytes = kBlockBytes;
  _slab.chunk_iova.clear();
  for (ULONG_PTR frame : _os->blocks)
    _slab.chunk_iova.push_back(static_cast<uint64_t>(frame) * kPageBytes);
  _slab.head_bytes = head_len;
  _slab.head_chunk_bytes = kPageBytes;
  _slab.head_iova.clear();
  for (ULONG_PTR frame : _os->head)
    _slab.head_iova.push_back(static_cast<uint64_t>(frame) * kPageBytes);
  if (!_dma->attach(_slab))
    return false;

  for (const DmaQuiesceOp &q : _dma->quiesce_ops()) {
    REGPROXY_OP op{};
    op.offset = q.reg;
    op.value = q.bits;
    op.width = 4;
    op.kind = q.set ? REGPROXY_SET_BITS : REGPROXY_CLEAR_BITS;
    _os->quiesce.push_back(op);
  }
  DWORD returned = 0;
  if (_os->quiesce.empty() ||
      !DeviceIoControl(
          _os->proxy, IOCTL_REGPROXY_SET_CLOSE_OPS, _os->quiesce.data(),
          static_cast<DWORD>(_os->quiesce.size() * sizeof(REGPROXY_OP)),
          nullptr, 0, &returned, nullptr)) {
    _logger->error("PcieTransport: could not arm the DMA stop the driver "
                   "replays on exit ({}) — refusing to enable DMA without it",
                   GetLastError());
    return false;
  }
  _logger->info("PcieTransport: DMA slab is {} locked 2 MiB blocks, physical "
                "0x{:x}..0x{:x}, and {} single pages below 4 GiB, used as bus "
                "addresses",
                _os->blocks.size(), _slab.chunk_iova.front(),
                _slab.chunk_iova.back() + kBlockBytes, _os->head.size());
  return true;
}

} /* namespace devourer */
