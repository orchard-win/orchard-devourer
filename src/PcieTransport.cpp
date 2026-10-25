#include "PcieTransport.h"

/* Bus-neutral half of the PCIe transport. The host backend that reaches the
 * device lives in PcieTransportVfio.cpp (Linux) or PcieTransportWin.cpp. */

#include "Event.h"
#if defined(DEVOURER_HAVE_JAGUAR2_8821C) && !defined(_WIN32)
#include "PcieDma88xx.h"
#define DEVOURER_PCIE_HAVE_88XX 1
#endif
#if defined(DEVOURER_HAVE_KESTREL_8852C)
#include "PcieDmaAx.h"
#endif

namespace devourer {

namespace {
constexpr uint16_t kPciDidRtl8852ce = 0xC852;
constexpr uint16_t kPciDidRtl8852be = 0xB852;
} /* namespace */

std::shared_ptr<PcieTransport> PcieTransport::Open(const std::string &bdf,
                                                   Logger_t logger) {
  return Open(bdf, std::move(logger), Config{});
}

std::shared_ptr<PcieTransport> PcieTransport::Open(const std::string &bdf,
                                                   Logger_t logger,
                                                   const Config &cfg) {
  std::shared_ptr<PcieTransport> t(new PcieTransport(logger, cfg));
  t->_bdf = bdf;
  if (!t->open_device(bdf))
    return nullptr;
  if (!t->setup_config_space())
    return nullptr;
  if (!t->select_dma_plane())
    return nullptr;
  if (!t->init_dma())
    return nullptr;
  if (cfg.use_msi && t->_dma->uses_msi() && !t->setup_msi())
    logger->warn("PcieTransport: MSI setup failed — RX falls back to polling");
  logger->info("PcieTransport: {} ready ({} plane, BAR2 {} KiB, DMA slab {} "
               "KiB @ IOVA 0x{:x}, RX {})",
               bdf, t->_dma->name(), t->_mmio_len / 1024, t->_slab.len / 1024,
               t->_slab.iova(0), t->msi_fd() >= 0 ? "MSI+eventfd" : "polled");
  return t;
}

bool PcieTransport::select_dma_plane() {
  switch (_pci_did) {
  case kPciDidRtl8852ce:
#if defined(DEVOURER_HAVE_KESTREL_8852C)
    _dma = std::make_unique<PcieDmaAx>(*this, _logger, _cfg.rx_poll_us,
                                       _cfg.tx_quick_submit);
    return true;
#else
    _logger->error("PcieTransport: RTL8852CE ({:04x}:{:04x}) needs "
                   "DEVOURER_KESTREL_8852C=ON",
                   _pci_vid, _pci_did);
    return false;
#endif
  case kPciDidRtl8852be:
#if defined(DEVOURER_HAVE_KESTREL_8852B)
    // DMA contract is ported. CreateRadioPcie still refuses live bring-up
    // until the BE power/PHY lifecycle and station identity have been ported.
    _dma = std::make_unique<PcieDmaAx>(*this, _logger, _cfg.rx_poll_us, false);
    return true;
#else
    _logger->error("PcieTransport: RTL8852BE needs DEVOURER_KESTREL_8852B=ON");
    return false;
#endif
  default:
#if defined(DEVOURER_PCIE_HAVE_88XX)
    /* The HalMAC 88xx ring plane; the factory still checks the chip-id. */
    _dma = std::make_unique<PcieDma88xx>(*this, _logger, _cfg.rx_ring_len,
                                         _cfg.rx_buf_size, _cfg.rx_poll_us);
    return true;
#else
    _logger->error("PcieTransport: PCI device {:04x}:{:04x} has no DMA plane "
                   "in this build",
                   _pci_vid, _pci_did);
    return false;
#endif
  }
}

bool PcieTransport::ltr_allowed() {
  if (!_pcie_cap)
    return false;
  uint16_t devctl2 = 0;
  if (!cfg_read(_pcie_cap + 0x28, &devctl2, 2) || devctl2 == 0xFFFF)
    return false;
  return (devctl2 & (1u << 10)) != 0; /* LTR Mechanism Enable */
}

void PcieTransport::warn_oob(uint32_t off) {
  if (!_warned_oob.exchange(true))
    _logger->warn("PcieTransport: register 0x{:x} is past the {} KiB BAR2 — "
                  "access dropped (further ones silent)",
                  off, _mmio_len / 1024);
}

bool PcieTransport::write32_wide(uint32_t addr, uint32_t v) {
  if (addr + 4 > _mmio_len) {
    warn_oob(addr);
    return false;
  }
  mmio_write<uint32_t>(addr, v);
  return true;
}

uint32_t PcieTransport::read32_wide(uint32_t addr) {
  if (addr + 4 > _mmio_len) {
    warn_oob(addr);
    return 0;
  }
  return mmio_read<uint32_t>(addr);
}

uint32_t PcieTransport::read_split(uint32_t off, size_t n) {
  /* Misaligned access: 16-bit halves when the offset allows, else bytes. */
  uint32_t v = 0;
  size_t done = 0;
  while (done < n) {
    const uint32_t a = off + static_cast<uint32_t>(done);
    const size_t width = ((a & 1) == 0 && n - done >= 2) ? 2 : 1;
    v |= reg_read(a, width) << (8 * done);
    done += width;
  }
  return v;
}

void PcieTransport::write_split(uint32_t off, size_t n, uint32_t v) {
  size_t done = 0;
  while (done < n) {
    const uint32_t a = off + static_cast<uint32_t>(done);
    const size_t width = ((a & 1) == 0 && n - done >= 2) ? 2 : 1;
    reg_write(a, width, (v >> (8 * done)) & (width == 2 ? 0xFFFFu : 0xFFu));
    done += width;
  }
}

int PcieTransport::tx_sync(uint8_t ep, uint8_t *buf, size_t len,
                           int timeout_ms) {
  _tx_submitted.fetch_add(1, std::memory_order_relaxed);
  int rc = _dma->tx_submit(ep, buf, len, timeout_ms);
  if (rc < 0) {
    _tx_failed.fetch_add(1, std::memory_order_relaxed);
    _tx_last_rc.store(rc, std::memory_order_relaxed);
    devourer::Ev(_logger->events(), "tx.fail").f("rc", rc).f("timeout", true);
  }
  return rc;
}

void PcieTransport::set_tx_kick_deferred(bool deferred) {
  if (_dma) _dma->set_tx_kick_deferred(deferred);
}

bool PcieTransport::tx_kick() { return _dma && _dma->tx_kick(); }

devourer::TxStats PcieTransport::tx_stats() const {
  devourer::TxStats s;
  s.submitted = _tx_submitted.load(std::memory_order_relaxed);
  s.failed = _tx_failed.load(std::memory_order_relaxed);
  s.last_error_rc = _tx_last_rc.load(std::memory_order_relaxed);
  s.last_was_timeout = s.failed != 0;
  if (_dma) _dma->add_tx_reports(s);
  return s;
}

} /* namespace devourer */
