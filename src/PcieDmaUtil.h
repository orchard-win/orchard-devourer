#pragma once

/* Small helpers shared by the PCIe DMA planes. */

#include <cstddef>

#include "PreciseDelay.h"

namespace devourer::pcie_dma {

constexpr size_t kPageSize = 4096;
inline size_t page_align(size_t v) {
  return (v + kPageSize - 1) & ~(kPageSize - 1);
}
inline void sleep_us(unsigned us) { devourer::precise_delay_us(us); }

} /* namespace devourer::pcie_dma */
