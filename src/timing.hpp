// SPDX-License-Identifier: LGPL-3.0-only
#pragma once
#include <chrono>

namespace emcl3d::detail {
using SteadyClock = std::chrono::steady_clock;

inline double elapsed_ms(SteadyClock::time_point started) {
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - started).count();
}
}  // namespace emcl3d::detail
