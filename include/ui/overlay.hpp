// include/ui/overlay.hpp
#pragma once
/**
 * @file overlay.hpp
 * @brief Draw-only overlay utilities.
 */

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "common/types.hpp"

namespace traffic::ui {

class Overlay {
public:
  struct Options {
    int font_face{cv::FONT_HERSHEY_SIMPLEX};
    double font_scale{0.6};
    int thickness{2};
    int text_line_height{18};
    bool draw_footpoint{true};
  };

  Overlay();
  explicit Overlay(const Options& opt);

  void drawTracks(cv::Mat& frame_bgr, const std::vector<traffic::Track>& tracks) const;
  void drawFootpoints(cv::Mat& frame_bgr, const std::vector<traffic::TrackFoot>& feet) const;
  void drawSpeeds(cv::Mat& frame_bgr, const std::vector<traffic::SpeedResult>& speeds) const;

  void drawAll(cv::Mat& frame_bgr,
               const std::vector<traffic::Track>& tracks,
               const std::vector<traffic::TrackFoot>& feet,
               const std::vector<traffic::SpeedResult>& speeds) const;

private:
  Options opt_;

  static cv::Rect toCvRect_(const traffic::BBox& b);
  static cv::Point clampPoint_(const cv::Point& p, int w, int h);
};

} // namespace traffic::ui
