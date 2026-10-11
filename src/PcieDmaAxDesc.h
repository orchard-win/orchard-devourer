#pragma once

/* The AX (HAXI) descriptor encodings that carry a bus address, apart from the
 * plane so they can be checked without hardware. Fields follow rtw89 pci.h. */

#include <cstdint>

namespace devourer::pcie_ax {

/* The DMA mask rtw89 sets on the 8852C, which needs no enabling: only the
 * 8852A/8852B generation has to be switched to 64-bit addressing first. */
constexpr unsigned kAddressBits = 36;
constexpr uint64_t kAddressLimit = uint64_t{1} << kAddressBits;
constexpr uint64_t kLowAddressLimit = uint64_t{1} << 32;

constexpr unsigned kBdOptAddressHighShift = 6; /* *_OPT_DMA_HI [13:6] */
constexpr uint32_t kAddrInfoBytes = 6;
constexpr uint16_t kAddrInfoLengthMask = 0x7FF;    /* B_PCIADDR_LEN_V1_MASK */
constexpr unsigned kAddrInfoAddressHighShift = 11; /* B_PCIADDR_HIGH_SEL_V1 */
constexpr uint16_t kAddrInfoLast = 1u << 15;       /* B_PCIADDR_LS_V1_MASK */

/* Address bits [35:32], the part that does not fit a descriptor's 32-bit
 * address field. */
constexpr uint16_t address_high(uint64_t dma) {
  return static_cast<uint16_t>((dma >> 32) & 0xF);
}

/* A TX or RX buffer descriptor: {le16 length, le16 opt, le32 dma}, written
 * as volatile byte stores because the chip reads it. */
inline void bd_write(volatile uint8_t *bd, uint16_t length, uint16_t opt,
                     uint64_t dma) {
  opt = static_cast<uint16_t>(opt |
                              (address_high(dma) << kBdOptAddressHighShift));
  bd[0] = static_cast<uint8_t>(length);
  bd[1] = static_cast<uint8_t>(length >> 8);
  bd[2] = static_cast<uint8_t>(opt);
  bd[3] = static_cast<uint8_t>(opt >> 8);
  bd[4] = static_cast<uint8_t>(dma);
  bd[5] = static_cast<uint8_t>(dma >> 8);
  bd[6] = static_cast<uint8_t>(dma >> 16);
  bd[7] = static_cast<uint8_t>(dma >> 24);
}

/* One addr_info_v1 entry of a WD page: {le16 length_opt, le16 dma[15:0],
 * le16 dma[31:16]}. `last` marks the final piece of the frame. */
inline void addr_info_write(uint8_t *entry, uint32_t length, bool last,
                            uint64_t dma) {
  const uint16_t length_opt =
      static_cast<uint16_t>((length & kAddrInfoLengthMask) |
                            (address_high(dma) << kAddrInfoAddressHighShift) |
                            (last ? kAddrInfoLast : 0));
  entry[0] = static_cast<uint8_t>(length_opt);
  entry[1] = static_cast<uint8_t>(length_opt >> 8);
  entry[2] = static_cast<uint8_t>(dma);
  entry[3] = static_cast<uint8_t>(dma >> 8);
  entry[4] = static_cast<uint8_t>(dma >> 16);
  entry[5] = static_cast<uint8_t>(dma >> 24);
}

} /* namespace devourer::pcie_ax */
