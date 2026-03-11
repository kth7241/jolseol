// include/track/tracker.hpp
#pragma once
/**
 * @file tracker.hpp
 * @brief Tracking interface + IOU 기반 최소 추적기 선언
 *
 * 설계 의도:
 * - detector가 만든 Detection 목록에 track_id를 붙여 Track 목록으로 변환한다.
 * - 다른 모듈(speed/ui/ipm/zone)에 대한 의존성을 만들지 않는다.
 * - 내부 상태는 tracker 안에만 숨긴다.
 *
 * 이번 버전에서 보강한 점:
 * - 같은 class_id끼리만 매칭하는 class gating
 * - bbox EMA smoothing
 * - 너무 금방 사라지는 잡음 track를 거르기 위한 min_hits
 */

#include <cstdint>
#include <memory>
#include <vector>

#include "common/types.hpp"

namespace traffic {

class ITracker {
public:
  virtual ~ITracker() = default;

  /**
   * @brief detection 목록을 받아 tracking 결과를 반환한다.
   * @param dets detector 출력
   * @param ts 현재 timestamp(ns)
   * @return 현재 시점의 활성 track 목록
   */
  virtual std::vector<Track> update(const std::vector<Detection>& dets,
                                    TimestampNs ts) = 0;

  /// 스트림 재시작 시 내부 상태 초기화
  virtual void reset() = 0;
};

class IouTracker final : public ITracker {
public:
  struct Options {
    float iou_threshold{0.3f};     ///< IOU 매칭 임계값
    int max_age{10};               ///< 몇 프레임까지 미매칭 허용할지
    int min_hits{2};               ///< 몇 번 이상 관측되어야 안정된 track로 볼지
    float bbox_ema_alpha{0.7f};    ///< bbox smoothing 계수 (클수록 현재 bbox 비중 큼)
    bool class_aware{true};        ///< class_id가 다르면 매칭하지 않음
  };

  explicit IouTracker(const Options& opt);
  IouTracker(float iou_threshold, int max_age);

  std::vector<Track> update(const std::vector<Detection>& dets,
                            TimestampNs ts) override;

  void reset() override;

private:
  struct TrackState {
    Track pub;            ///< 외부로 내보낼 공용 Track 상태
    int hit_count{0};     ///< 누적 매칭 횟수
    bool confirmed{false};///< min_hits 이상 만족 여부
  };

  Options opt_{};
  int next_track_id_{1};
  std::vector<TrackState> tracks_; ///< tracker 내부 활성 상태

  static float iou_(const BBox& a, const BBox& b);
  static float intersectionArea_(const BBox& a, const BBox& b);
  static float area_(const BBox& a);
  static BBox smoothBBox_(const BBox& prev, const BBox& cur, float alpha);

  bool classMatch_(const TrackState& tr, const Detection& det) const;
  void purgeOld_();
};

std::unique_ptr<ITracker> CreateIouTracker(float iou_threshold, int max_age);
std::unique_ptr<ITracker> CreateIouTracker(const IouTracker::Options& opt);

} // namespace traffic
