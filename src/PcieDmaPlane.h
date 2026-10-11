#pragma once

/* IPcieDmaPlane — the frame plane of the PCIe transport, one implementation
 * per DMA engine generation. The vfio/BAR/config-space/DMA-slab/MSI plumbing
 * in PcieTransport is chip-agnostic; what differs between a HalMAC 11ac part
 * (RTL8821CE: 88xx buffer-descriptor rings, rtw88) and an AX part (RTL8852CE:
 * HAXI TXBD/WD pages + RXQ/RPQ rings, rtw89) is everything below this
 * interface. PcieTransport::Open picks the plane from the PCI device id and
 * forwards tx_sync / rx_loop / hci_setup to it.
 *
 * A plane carves its rings and buffers out of the one IOMMU-mapped slab the
 * transport allocates (slab_bytes() tells the transport how much), and reaches
 * registers through PcieTransport::mmio_read/mmio_write. */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "TxStats.h"

namespace devourer {

/* DMA memory handed to a plane. With chunk_bytes set it is bus-contiguous only
 * inside a chunk, and no DMA object may straddle a chunk boundary. With
 * head_bytes set, that many leading bytes come in smaller chunks of their own
 * and chunk_iova describes what follows them. */
struct DmaSlab {
  uint8_t *va = nullptr;
  size_t len = 0;
  size_t chunk_bytes = 0;
  std::vector<uint64_t> chunk_iova;
  size_t head_bytes = 0;
  size_t head_chunk_bytes = 0;
  std::vector<uint64_t> head_iova;

  uint64_t iova(size_t off) const {
    if (off < head_bytes)
      return head_iova[off / head_chunk_bytes] + off % head_chunk_bytes;
    off -= head_bytes;
    return chunk_bytes ? chunk_iova[off / chunk_bytes] + off % chunk_bytes
                       : chunk_iova[0] + off;
  }
};

/* A register bit operation that stops a plane's DMA engine, for a host that
 * can replay it if the process dies with DMA live. */
struct DmaQuiesceOp {
  uint32_t reg;
  uint32_t bits;
  bool set; /* false = clear */
};

class IPcieDmaPlane {
public:
  virtual ~IPcieDmaPlane() = default;

  virtual const char *name() const = 0;

  /* DMA slab bytes this plane needs (rings + buffers). Called before attach. */
  virtual size_t slab_bytes() const = 0;
  /* Width of the bus addresses this plane can hand the chip. Every address
   * in the slab must fit. */
  virtual unsigned dma_address_bits() const { return 32; }
  /* How many leading bytes of the slab must lie below 4 GiB even when
   * dma_address_bits() allows more. */
  virtual size_t low_slab_bytes() const { return 0; }
  /* Carve the slab into rings/buffers and arm the RX descriptors. Register
   * programming is NOT done here — that is setup_rings(), which runs at the
   * generation's intf_pre_init slot. Returns false for a slab layout the
   * plane cannot use (a chunked slab on a plane that needs one run). */
  virtual bool attach(const DmaSlab &slab) = 0;

  /* 32-bit register operations that stop this plane's DMA, in order. */
  virtual std::vector<DmaQuiesceOp> quiesce_ops() const { return {}; }

  /* Program the ring base/num/index registers and reset the host indices
   * (rtw88 rtw_pci_reset_buf_desc / rtw89 rtw89_pci_reset_trx_rings). Where
   * this sits in the bring-up is per generation: before power-on on the 88xx
   * plane, after dmac_func_pre_en and before FWDL on the AX plane. */
  virtual void setup_rings() = 0;

  /* Synchronous TX. `hint` is the HAL's queue handle: ignored by the 88xx
   * plane (it routes on the descriptor QSEL byte) and the AX DMA channel on
   * the AX plane. Returns bytes submitted or a negative error. */
  virtual int tx_submit(uint8_t hint, uint8_t *buf, size_t len,
                        int timeout_ms) = 0;

  /* Adds what the chip reported about sent frames (TxStats::reported and
   * given_up) on a plane whose bus carries such reports. */
  virtual void add_tx_reports(TxStats &) const {}

  /* Blocking RX reap loop until should_stop(); each delivery is one chip
   * packet (descriptor-headed), never a USB-style aggregate. */
  virtual void rx_loop(const std::function<void(const uint8_t *, int)> &on_data,
                       const std::function<bool()> &should_stop) = 0;

  /* Whether this plane consumes the vfio MSI eventfd in its RX loop. The
   * transport registers the vector only for a plane that says so, so a
   * polling plane neither arms an unused IRQ nor reports "MSI" as its RX
   * mechanism. */
  virtual bool uses_msi() const { return false; }
  /* Mask the plane's RX interrupt sources (teardown, before the MSI vector is
   * dropped). Default: nothing was enabled. */
  virtual void irq_mask() {}

  /* Register-plane filter for 16-bit-addressed accesses: false refuses the
   * access (the 88xx plane rejects the USB-only 0xFE00 page). */
  virtual bool reg_allowed(uint32_t reg) const {
    (void)reg;
    return true;
  }
};

} /* namespace devourer */
