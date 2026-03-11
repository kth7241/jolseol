// src/track/iou_tracker.cpp
/**
 * @file iou_tracker.cpp
 * @brief Minimal IOU-based tracker implementation (MVP).
 *
 * 좌표계:
 * - 모든 bbox는 이미지 픽셀 좌표(px), (x,y)=top-left, w/h>0
 *
 * 시간:
 * - TimestampNs는 steady_clock 기반 ns 가정
 * - 이 구현은 dt를 직접 사용하지 않고 "프레임 기반 max_age"로 purge한다.
 */

#include "track/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace traffic {

IouTracker::IouTracker(float iou_threshold, int max_age)
  : iou_threshold_(iou_threshold), max_age_(max_age) {
  if (iou_threshold_ < 0.f) iou_threshold_ = 0.f;
  if (iou_threshold_ > 1.f) iou_threshold_ = 1.f;
  if (max_age_ < 0) max_age_ = 0;
}

void IouTracker::reset() {
  tracks_.clear();
  next_track_id_ = 1;
}

std::unique_ptr<ITracker> CreateIouTracker(float iou_threshold, int max_age) {
  return std::make_unique<IouTracker>(iou_threshold, max_age);
}

std::vector<Track> IouTracker::update(const std::vector<Detection>& dets,
                                      TimestampNs ts) {
  // 1) mark all tracks as unmatched initially
  // We'll use two boolean arrays: track_matched[i], det_matched[j]
  const int T = static_cast<int>(tracks_.size());
  const int D = static_cast<int>(dets.size());

  std::vector<bool> track_matched(T, false);
  std::vector<bool> det_matched(D, false);

  // 2) Greedy matching:
  // For each detection, find best track above threshold (not matched yet).
  // (Alternative: track->det loop. Either is okay for MVP.)
  for (int j = 0; j < D; ++j) {
    const auto& det = dets[j];
    if (!det.bbox_img.valid()) {
      det_matched[j] = true; // ignore invalid detection
      continue;
    }

    float best_iou = -1.f;
    int best_i = -1;

    for (int i = 0; i < T; ++i) {
      if (track_matched[i]) continue;
      const auto& tr = tracks_[i];
      if (!tr.bbox_img.valid()) continue;

      // Optional: class gating (same class only). For MVP, we keep it simple:
      // if (tr.class_id != det.class_id) continue;

      const float v = iou_(tr.bbox_img, det.bbox_img);
      if (v > best_iou) {
        best_iou = v;
        best_i = i;
      }
    }

    if (best_i >= 0 && best_iou >= iou_threshold_) {
      // Match det j with track best_i
      Track& tr = tracks_[best_i];
      tr.ts_ns = ts;
      tr.bbox_img = det.bbox_img;
      tr.class_id = det.class_id;
      tr.score = det.score;

      tr.age += 1;
      tr.lost = 0;

      track_matched[best_i] = true;
      det_matched[j] = true;
    }
  }

  // 3) Unmatched existing tracks: increase lost
  for (int i = 0; i < T; ++i) {
    if (!track_matched[i]) {
      tracks_[i].lost += 1;
      tracks_[i].age += 1;
      // Keep ts_ns as last updated time (or update to current ts). We keep last updated.
    }
  }

  // 4) Unmatched detections -> new tracks
  for (int j = 0; j < D; ++j) {
    if (det_matched[j]) continue;
    const auto& det = dets[j];
    if (!det.bbox_img.valid()) continue;

    Track tr;
    tr.ts_ns = ts;
    tr.track_id = next_track_id_++;
    tr.bbox_img = det.bbox_img;
    tr.class_id = det.class_id;
    tr.score = det.score;

    tr.age = 1;
    tr.lost = 0;

    tracks_.push_back(tr);
  }

  // 5) Purge old tracks
  purgeOld_();

  // 6) Return active tracks (copy)
  // For MVP simplicity we return tracks_ directly.
  return tracks_;
}

void IouTracker::purgeOld_() {
  // Remove tracks whose lost > max_age_
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
                     [&](const Track& t) { return t.lost > max_age_; }),
      tracks_.end());
}

// --------------------------
// IOU helpers
// --------------------------
float IouTracker::area_(const BBox& a) {
  if (!a.valid()) return 0.f;
  return a.w * a.h;
}

float IouTracker::intersectionArea_(const BBox& a, const BBox& b) {
  if (!a.valid() || !b.valid()) return 0.f;

  const float ax1 = a.x;
  const float ay1 = a.y;
  const float ax2 = a.x2();
  const float ay2 = a.y2();

  const float bx1 = b.x;
  const float by1 = b.y;
  const float bx2 = b.x2();
  const float by2 = b.y2();

  const float ix1 = std::max(ax1, bx1);
  const float iy1 = std::max(ay1, by1);
  const float ix2 = std::min(ax2, bx2);
  const float iy2 = std::min(ay2, by2);

  const float iw = ix2 - ix1;
  const float ih = iy2 - iy1;

  if (iw <= 0.f || ih <= 0.f) return 0.f;
  return iw * ih;
}

float IouTracker::iou_(const BBox& a, const BBox& b) {
  const float inter = intersectionArea_(a, b);
  const float ua = area_(a);
  const float ub = area_(b);
  const float uni = ua + ub - inter;
  if (uni <= 0.f) return 0.f;
  return inter / uni;
}

} // namespace traffic