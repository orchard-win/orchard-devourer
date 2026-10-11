#include "PreciseDelay.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace devourer {

namespace {

/* Below this a timer's wake-up latency exceeds the wait itself. */
constexpr uint32_t kSpinBelowUs = 100;

/* High-resolution timers (Windows 10 1803+) wake within about half a
 * millisecond whatever the system tick is. One per thread, closed with it. */
HANDLE thread_timer() {
  struct Timer {
    HANDLE handle = CreateWaitableTimerExW(
        nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
        TIMER_ALL_ACCESS);
    ~Timer() {
      if (handle)
        CloseHandle(handle);
    }
  };
  thread_local Timer timer;
  return timer.handle;
}

} /* namespace */

void precise_delay_us(uint32_t us) {
  if (us < kSpinBelowUs) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::microseconds(us);
    while (std::chrono::steady_clock::now() < deadline)
      YieldProcessor();
    return;
  }
  LARGE_INTEGER due;
  due.QuadPart = -static_cast<LONGLONG>(us) * 10; /* relative, 100 ns units */
  const HANDLE timer = thread_timer();
  if (timer && SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE))
    WaitForSingleObject(timer, INFINITE);
  else
    Sleep((us + 999) / 1000);
}

} /* namespace devourer */
