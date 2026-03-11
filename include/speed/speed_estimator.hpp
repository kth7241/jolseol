#pragma once
/**
 * @file speed_estimator.hpp
 * @brief IPM footpoint 기반 속도 추정기
 *
 * 설계 의도:
 * - TrackFoot만 입력으로 받는다.
 * - 이전 위치/시간 상태는 estimator 내부에만 보관한다.
 * - tracker / ui / zone 내부 구현을 몰라도 동작한다.
 */

#include <vector>

#include "common/types.hpp"
#include "speed/track_state_table.hpp"

namespace traffic {

class SpeedEstimator {
public:
  struct Options {
    float ema_alpha{0.3f};           ///< 속도 EMA smoothing 계수
    float dt_min{0.01f};             ///< 너무 짧은 dt 방지용 최소값 [s]
    float dt_max{0.2f};              ///< 너무 긴 dt 방지용 최대값 [s]
    float max_speed_kmh{200.f};      ///< 비현실적 속도 clamp [km/h]
    float purge_timeout_sec{1.0f};   ///< 오래 안 보인 track 상태 제거 기준 [s]
    float min_displacement_px{1.5f}; ///< 작은 떨림을 0으로 보는 최소 이동량 [px]
    bool require_zone{false};        ///< true면 zone이 없을 때 quality를 NoZone으로 표기
  };

  explicit SpeedEstimator(const Options& opt);

  /**
   * @brief TrackFoot 목록으로부터 속도 결과를 계산한다.
   * @param feet footpoint 목록 (IPM pixel 좌표)
   * @param meters_per_pixel IPM scale [m/px]
   */
  std::vector<SpeedResult> compute(const std::vector<TrackFoot>& feet,
                                   float meters_per_pixel);

private:
  Options opt_;
  TrackStateTable states_;
};

} // namespace traffic
