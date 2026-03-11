#pragma once
#include <cmath>
#include <limits>

namespace traffic {

/**
 * @brief EMA (Exponential Moving Average)
 * @param prev 이전 값
 * @param cur  현재 값
 * @param alpha 0~1 (클수록 현재값 비중 ↑)
 */
inline float ema(float prev, float cur, float alpha) {
  if (std::isnan(prev)) return cur;
  return alpha * cur + (1.f - alpha) * prev;
}

/**
 * @brief 값 유효성 검사
 */
inline bool isFinite(float v) {
  return std::isfinite(v);
}

} // namespace traffic
