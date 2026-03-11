#pragma once
#include <unordered_map>
#include <opencv2/core.hpp>
#include <cstdint>

namespace traffic {

/**
 * @brief track_id별 상태 저장 구조체
 *
 * 좌표계:
 * - prev_foot_ipm: IPM 좌표계 [pixel]
 *
 * 단위:
 * - prev_ts_ns: nanoseconds (steady_clock)
 * - prev_speed_kmh: km/h
 */
struct TrackState {
  cv::Point2f prev_foot_ipm {0.f, 0.f};
  int64_t     prev_ts_ns = 0;
  float       prev_speed_kmh = 0.f;
  bool        initialized = false;
};

/**
 * @brief track_id -> TrackState 저장소
 *
 * 역할:
 * - speed estimator 내부 상태 보관
 * - 오래 업데이트 안 된 track purge 지원
 */
class TrackStateTable {
public:
  TrackState& get(int track_id);

  bool has(int track_id) const;

  void erase(int track_id);

  /**
   * @brief 일정 시간 이상 업데이트 없는 track 삭제
   * @param now_ns 현재 timestamp [ns]
   * @param timeout_ns timeout 기준 [ns]
   */
  void purge(int64_t now_ns, int64_t timeout_ns);

private:
  std::unordered_map<int, TrackState> table_;
};

} // namespace traffic
