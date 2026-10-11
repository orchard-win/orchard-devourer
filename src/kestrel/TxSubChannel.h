#pragma once

/* Where a frame narrower than the channel sits inside it: the TXSC codes the
 * MAC takes in R_AX_TX_SUB_CARRIER_VALUE, as rtw89_phy_get_txsc computes them
 * for a channel of 20, 40 or 80 MHz. Apart from the HAL so it can be checked
 * without hardware. */

#include <cstdint>

namespace kestrel {

/* The code for a `frame_mhz` wide frame in a `channel_mhz` wide channel whose
 * primary 20 MHz is channel number `primary` and whose centre is `center`.
 * 0 when the frame fills the channel. */
constexpr uint8_t tx_sub_channel(uint8_t primary, uint8_t center,
                                 unsigned channel_mhz, unsigned frame_mhz) {
  if (channel_mhz == frame_mhz || channel_mhz == 20)
    return 0;
  const bool above = primary > center;
  if (channel_mhz == 40)
    return above ? 1 : 2;
  if (channel_mhz == 80) {
    if (frame_mhz == 20)
      return static_cast<uint8_t>(above ? (primary - center) >> 1
                                        : ((center - primary) >> 1) + 1);
    return above ? 9 : 10;
  }
  return 0;
}

/* The value of R_AX_TX_SUB_CARRIER_VALUE for such a channel: the 20 MHz code
 * in bits 3:0 and the 40 MHz code in bits 7:4. */
constexpr uint32_t tx_sub_channels(uint8_t primary, uint8_t center,
                                   unsigned channel_mhz) {
  const uint32_t in_20 = tx_sub_channel(primary, center, channel_mhz, 20);
  const uint32_t in_40 =
      channel_mhz > 40 ? tx_sub_channel(primary, center, channel_mhz, 40) : 0;
  return in_20 | in_40 << 4;
}

} /* namespace kestrel */
