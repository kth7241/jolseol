// traffic_monitor/include/common/math.hpp
#pragma once

#include <algorithm>
#include <cmath>

namespace traffic {

template <typename T>
inline T clamp(T v, T lo, T hi) {
  return std::max(lo, std::min(v, hi));
}

// Exponential Moving Average
// alpha: 0~1 (0이면 이전값 유지, 1이면 새값 즉시 반영)
inline float ema(float prev, float cur, float alpha) {
  alpha = clamp(alpha, 0.0f, 1.0f);
  return prev * (1.0f - alpha) + cur * alpha;
}

// dt(ns)를 seconds로 바꾸되, 너무 작거나 크면 clamp해서 폭주 방지
inline double safeDtSec(int64_t dt_ns, double min_dt = 1e-3, double max_dt = 0.2) {
  const double dt = static_cast<double>(dt_ns) * 1e-9;
  return clamp(dt, min_dt, max_dt);
}

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

} // namespace traffic
