// src/track/iou_tracker.cpp
/**
 * @file iou_tracker.cpp
 * @brief IOU 기반 최소 추적기 구현
 *
 * 구현 포인트:
 * - Hungarian 같은 무거운 최적화는 아직 넣지 않음
 * - 대신 실사용에 필요한 최소 안정화만 추가
 *   1) class gating
 *   2) bbox EMA smoothing
 *   3) min_hits 기반 confirmed track 필터링
 */

#include "track/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace traffic {

namespace {
inline float clamp01(float v) {
  return std::max(0.f, std::min(1.f, v));
}
} // namespace

IouTracker::IouTracker(const Options& opt) : opt_(opt) {
  opt_.iou_threshold = clamp01(opt_.iou_threshold);
  if (opt_.max_age < 0) opt_.max_age = 0;
  if (opt_.min_hits < 1) opt_.min_hits = 1;
  opt_.bbox_ema_alpha = clamp01(opt_.bbox_ema_alpha);
}

IouTracker::IouTracker(float iou_threshold, int max_age)
    : IouTracker(Options{.iou_threshold = iou_threshold,
                         .max_age = max_age,
                         .min_hits = 2,
                         .bbox_ema_alpha = 0.7f,
                         .class_aware = true}) {}

void IouTracker::reset() {
  tracks_.clear();
  next_track_id_ = 1;
}

std::unique_ptr<ITracker> CreateIouTracker(float iou_threshold, int max_age) {
  return std::make_unique<IouTracker>(iou_threshold, max_age);
}

std::unique_ptr<ITracker> CreateIouTracker(const IouTracker::Options& opt) {
  return std::make_unique<IouTracker>(opt);
}

std::vector<Track> IouTracker::update(const std::vector<Detection>& dets,
                                      TimestampNs ts) {
  const int T = static_cast<int>(tracks_.size());
  const int D = static_cast<int>(dets.size());

  std::vector<bool> track_matched(T, false);
  std::vector<bool> det_matched(D, false);

  // detection 하나마다 "아직 안 잡힌 기존 track 중 가장 IOU가 큰 것"을 찾는다.
  for (int j = 0; j < D; ++j) {
    const auto& det = dets[j];
    if (!det.bbox_img.valid()) {
      det_matched[j] = true;
      continue;
    }

    float best_iou = -1.f;
    int best_i = -1;

    for (int i = 0; i < T; ++i) {
      if (track_matched[i]) continue;
      const auto& tr = tracks_[i];
      if (!tr.pub.bbox_img.valid()) continue;
      if (!classMatch_(tr, det)) continue;

      const float v = iou_(tr.pub.bbox_img, det.bbox_img);
      if (v > best_iou) {
        best_iou = v;
        best_i = i;
      }
    }

    if (best_i >= 0 && best_iou >= opt_.iou_threshold) {
      TrackState& tr = tracks_[best_i];

      // bbox는 detector jitter를 조금 줄이기 위해 EMA smoothing 적용
      const BBox smoothed = smoothBBox_(tr.pub.bbox_img, det.bbox_img, opt_.bbox_ema_alpha);

      tr.pub.ts_ns = ts;
      tr.pub.bbox_img = smoothed;
      tr.pub.class_id = det.class_id;
      tr.pub.score = det.score;
      tr.pub.age += 1;
      tr.pub.lost = 0;

      tr.hit_count += 1;
      if (tr.hit_count >= opt_.min_hits) {
        tr.confirmed = true;
      }

      track_matched[best_i] = true;
      det_matched[j] = true;
    }
  }

  // 매칭 실패한 기존 track는 lost만 증가시킨다.
  for (int i = 0; i < T; ++i) {
    if (!track_matched[i]) {
      tracks_[i].pub.lost += 1;
      tracks_[i].pub.age += 1;
    }
  }

  // 새 detection은 새 track로 만든다.
  for (int j = 0; j < D; ++j) {
    if (det_matched[j]) continue;
    const auto& det = dets[j];
    if (!det.bbox_img.valid()) continue;

    TrackState tr;
    tr.pub.ts_ns = ts;
    tr.pub.track_id = next_track_id_++;
    tr.pub.bbox_img = det.bbox_img;
    tr.pub.class_id = det.class_id;
    tr.pub.score = det.score;
    tr.pub.age = 1;
    tr.pub.lost = 0;
    tr.hit_count = 1;
    tr.confirmed = (opt_.min_hits <= 1);

    tracks_.push_back(tr);
  }

  purgeOld_();

  // 외부에는 "확인된 track" 위주로 내보낸다.
  // 단, 아주 첫 단계 디버깅을 위해 age==1 새 track도 한 번은 보여줄 수 있게 둘 수도 있지만,
  // speed 계산 안정성을 위해 여기서는 confirmed만 반환한다.
  std::vector<Track> out;
  out.reserve(tracks_.size());
  for (const auto& tr : tracks_) {
    if (!tr.confirmed) continue;
    out.push_back(tr.pub);
  }
  return out;
}

bool IouTracker::classMatch_(const TrackState& tr, const Detection& det) const {
  if (!opt_.class_aware) return true;
  if (tr.pub.class_id < 0 || det.class_id < 0) return true;
  return tr.pub.class_id == det.class_id;
}

void IouTracker::purgeOld_() {
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
                     [&](const TrackState& t) { return t.pub.lost > opt_.max_age; }),
      tracks_.end());
}

float IouTracker::area_(const BBox& a) {
  if (!a.valid()) return 0.f;
  return a.w * a.h;
}

float IouTracker::intersectionArea_(const BBox& a, const BBox& b) {
  if (!a.valid() || !b.valid()) return 0.f;

  const float ix1 = std::max(a.x, b.x);
  const float iy1 = std::max(a.y, b.y);
  const float ix2 = std::min(a.x2(), b.x2());
  const float iy2 = std::min(a.y2(), b.y2());

  const float iw = ix2 - ix1;
  const float ih = iy2 - iy1;
  if (iw <= 0.f || ih <= 0.f) return 0.f;
  return iw * ih;
}

float IouTracker::iou_(const BBox& a, const BBox& b) {
  const float inter = intersectionArea_(a, b);
  const float uni = area_(a) + area_(b) - inter;
  if (uni <= 0.f) return 0.f;
  return inter / uni;
}

BBox IouTracker::smoothBBox_(const BBox& prev, const BBox& cur, float alpha) {
  if (!prev.valid()) return cur;
  const float a = clamp01(alpha);

  BBox out;
  out.x = (a * cur.x) + ((1.f - a) * prev.x);
  out.y = (a * cur.y) + ((1.f - a) * prev.y);
  out.w = (a * cur.w) + ((1.f - a) * prev.w);
  out.h = (a * cur.h) + ((1.f - a) * prev.h);
  return out;
}

} // namespace traffic
