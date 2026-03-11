// traffic_monitor/include/common/clock.hpp
#pragma once


#include <cstdint>
#include <chrono>

namespace traffic {

inline uint64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}


// steady_clock 기반 ns timestamp를 제공한다.
// 시스템 시간 변경(NTP 등)의 영향을 받지 않아 dt 계산에 안전하다.
class Clock {
public:
  using Steady = std::chrono::steady_clock;

  // 현재 시각(ns)
  static int64_t nowNs();

  // dt 계산 유틸 (ns -> seconds)
  static double nsToSec(int64_t ns) { return static_cast<double>(ns) * 1e-9; }
};

} // namespace traffic



