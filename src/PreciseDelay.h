#pragma once

/* A delay that lasts about as long as it says: sleep_for rounds up to the
 * scheduler tick, which on Windows can turn a 50 us wait into 15.6 ms. */

#include <chrono>
#include <cstdint>
#include <thread>

namespace devourer {

#if defined(_WIN32)
void precise_delay_us(uint32_t us);
#else
inline void precise_delay_us(uint32_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}
#endif

} /* namespace devourer */
