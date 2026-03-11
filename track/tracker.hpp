// include/track/tracker.hpp
#pragma once
/**
 * @file tracker.hpp
 * @brief Tracking interface + minimal IOU-based tracker implementation declaration.
 *
 * 역할:
 * - Detector가 제공한 Detection 목록에 대해 "track_id"를 부여하여 Track 목록을 만든다.
 * - MVP 단계에서는 매우 단순한 IOU 매칭으로 동작한다.
 *
 * 약결합 규칙:
 * - 모듈 간 데이터는 include/common/types.hpp 타입만 사용한다.
 * - speed/ui/ipm/zone 등 다른 모듈 include 금지.
 */

#include <memory>
#include <vector>
#include <cstdint>

#include "common/types.hpp"

namespace traffic {

/**
 * @brief Tracker interface.
 *
 * 입력:
 * - dets: 특정 시각(ts)의 Detection 목록 (bbox_img는 이미지 픽셀 좌표)
 * - ts:   TimestampNs (steady_clock 기반 ns, 단조 증가 가정)
 *
 * 출력:
 * - tracks: 동일 시각(ts)에 대한 Track 목록 (track_id 포함)
 */
class ITracker {
public:
  virtual ~ITracker() = default;

  virtual std::vector<Track> update(const std::vector<Detection>& dets,
                                    TimestampNs ts) = 0;

  /// Optional: reset internal states (e.g., when stream restarts)
  virtual void reset() = 0;
};

/**
 * @brief Minimal IOU-based tracker (MVP용).
 *
 * 동작 개요:
 * 1) 각 detection에 대해 기존 트랙들과 IOU를 계산
 * 2) IOU가 threshold 이상인 트랙 중 최고 IOU 트랙과 매칭(그리디 방식)
 * 3) 매칭된 트랙은 bbox/score/class 갱신 및 lost=0
 * 4) 매칭되지 않은 detection은 새 track_id를 생성
 * 5) 매칭되지 않은 기존 트랙은 lost 증가, lost > max_age면 삭제
 *
 * 한계(의도적으로 단순):
 * - Hungarian 최적 매칭이 아니라 greedy 매칭
 * - re-identification(재식별) 없음
 * - occlusion에 취약
 *
 * 그래도 MVP에서 "차량별 속도" 계산을 위한 track_id 부여에는 충분하다.
 */
class IouTracker final : public ITracker {
public:
  /**
   * @param iou_threshold   IOU 매칭 임계값 (예: 0.3~0.5)
   * @param max_age         연속으로 매칭 실패 허용 프레임 수 (예: 10)
   */
  IouTracker(float iou_threshold, int max_age);

  std::vector<Track> update(const std::vector<Detection>& dets,
                            TimestampNs ts) override;

  void reset() override;

private:
  float iou_threshold_{0.3f};
  int max_age_{10};

  int next_track_id_{1};
  std::vector<Track> tracks_;  // 내부 상태(활성 트랙)

  // -----------------------
  // Helpers (cpp에서 구현)
  // -----------------------
  static float iou_(const BBox& a, const BBox& b);
  static float intersectionArea_(const BBox& a, const BBox& b);
  static float area_(const BBox& a);

  void purgeOld_(); // lost > max_age_ 삭제
};

/**
 * @brief Factory helper for creating IOU tracker.
 */
std::unique_ptr<ITracker> CreateIouTracker(float iou_threshold, int max_age);

} // namespace traffic