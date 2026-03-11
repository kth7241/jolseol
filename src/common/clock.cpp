// traffic_monitor/src/common/clock.cpp
#include "common/clock.hpp"

namespace traffic {

int64_t Clock::nowNs() {
  const auto now = Steady::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

} // namespace traffic