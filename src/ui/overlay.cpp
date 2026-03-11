// src/ui/overlay.cpp
#include "ui/overlay.hpp"

#include <sstream>
#include <iomanip>
#include <unordered_map>

#include <opencv2/imgproc.hpp>

namespace traffic::ui {

Overlay::Overlay() : opt_(Options{}) {}

Overlay::Overlay(const Options& opt) : opt_(opt) {}

cv::Rect Overlay::toCvRect_(const traffic::BBox& b) {
  return cv::Rect(static_cast<int>(std::round(b.x)),
                  static_cast<int>(std::round(b.y)),
                  static_cast<int>(std::round(b.w)),
                  static_cast<int>(std::round(b.h)));
}

cv::Point Overlay::clampPoint_(const cv::Point& p, int w, int h) {
  return cv::Point(std::clamp(p.x, 0, std::max(0, w - 1)),
                   std::clamp(p.y, 0, std::max(0, h - 1)));
}

void Overlay::drawTracks(cv::Mat& frame_bgr, const std::vector<traffic::Track>& tracks) const {
  if (frame_bgr.empty()) return;

  for (const auto& t : tracks) {
    if (!t.bbox_img.valid()) continue;
    cv::Rect r = toCvRect_(t.bbox_img);
    r &= cv::Rect(0, 0, frame_bgr.cols, frame_bgr.rows);

    // bbox
    cv::rectangle(frame_bgr, r, cv::Scalar(0, 255, 0), opt_.thickness);

    // label
    std::ostringstream ss;
    ss << "ID:" << t.track_id;
    if (t.class_id >= 0) ss << " C:" << t.class_id;

    const auto org = clampPoint_(cv::Point(r.x, std::max(0, r.y - 4)), frame_bgr.cols, frame_bgr.rows);
    cv::putText(frame_bgr, ss.str(), org, opt_.font_face, opt_.font_scale,
                cv::Scalar(0, 255, 0), opt_.thickness, cv::LINE_AA);
  }
}

void Overlay::drawFootpoints(cv::Mat& frame_bgr, const std::vector<traffic::TrackFoot>& feet) const {
  if (frame_bgr.empty()) return;
  if (!opt_.draw_footpoint) return;

  for (const auto& f : feet) {
    cv::Point p(static_cast<int>(std::round(f.foot_img.x)),
                static_cast<int>(std::round(f.foot_img.y)));
    p = clampPoint_(p, frame_bgr.cols, frame_bgr.rows);
    cv::circle(frame_bgr, p, 4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

    std::ostringstream ss;
    ss << f.track_id;
    if (f.zone_id.has_value()) ss << " Z:" << *f.zone_id;
    const auto org = clampPoint_(p + cv::Point(6, -6), frame_bgr.cols, frame_bgr.rows);
    cv::putText(frame_bgr, ss.str(), org, opt_.font_face, 0.5,
                cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
  }
}

void Overlay::drawSpeeds(cv::Mat& frame_bgr, const std::vector<traffic::SpeedResult>& speeds) const {
  if (frame_bgr.empty()) return;

  // Build map for quick lookup by track_id
  std::unordered_map<int, traffic::SpeedResult> mp;
  mp.reserve(speeds.size());
  for (const auto& s : speeds) mp[s.track_id] = s;

  int y = 24;
  for (const auto& [id, s] : mp) {
    std::ostringstream ss;
    ss << "ID " << id << ": " << std::fixed << std::setprecision(1) << s.speed_kmh << " km/h";
    if (s.zone_id.has_value()) ss << " Z:" << *s.zone_id;
    if (s.lane_id.has_value()) ss << " L:" << *s.lane_id;

    cv::putText(frame_bgr, ss.str(), cv::Point(10, y), opt_.font_face, 0.6,
                cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
    y += opt_.text_line_height;
    if (y > frame_bgr.rows - 10) break;
  }
}

void Overlay::drawAll(cv::Mat& frame_bgr,
                      const std::vector<traffic::Track>& tracks,
                      const std::vector<traffic::TrackFoot>& feet,
                      const std::vector<traffic::SpeedResult>& speeds) const {
  drawTracks(frame_bgr, tracks);
  drawFootpoints(frame_bgr, feet);
  drawSpeeds(frame_bgr, speeds);
}

} // namespace traffic::ui
