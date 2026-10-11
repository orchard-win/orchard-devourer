#pragma once

/* Choosing DMA blocks by physical address, apart from the operating system
 * calls that allocate them so the choice can be checked without them. */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace devourer {

/* Takes the `need` lowest blocks out of `frames`, or none: all must end by
 * `limit_frame`, and the first `low_need` of them by `low_limit_frame`. */
inline std::vector<uintptr_t>
take_lowest_blocks(std::vector<uintptr_t> &frames, size_t need,
                   uintptr_t block_frames, uintptr_t limit_frame,
                   size_t low_need = 0, uintptr_t low_limit_frame = 0) {
  std::sort(frames.begin(), frames.end());
  auto fits = [&](size_t count, uintptr_t limit) {
    return count <= frames.size() &&
           (count == 0 || frames[count - 1] + block_frames <= limit);
  };
  std::vector<uintptr_t> taken;
  if (fits(need, limit_frame) && fits(low_need, low_limit_frame)) {
    taken.assign(frames.begin(), frames.begin() + need);
    frames.erase(frames.begin(), frames.begin() + need);
  }
  return taken;
}

} /* namespace devourer */
