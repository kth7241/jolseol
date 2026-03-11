// src/detect/detector_stub.cpp
#include "detect/detector.hpp"

#include <algorithm>
#include "common/clock.hpp"

namespace traffic::detect {

std::unique_ptr<IDetector> CreateDetectorStub() {
  return std::make_unique<DetectorStub>();
}

std::vector<traffic::Detection> DetectorStub::infer(const cv::Mat& frame) {
  std::vector<traffic::Detection> out;
  if (frame.empty()) return out;

  const int W = frame.cols;
  const int H = frame.rows;

  const float bw = std::max(8.0f, params_.box_w_ratio * static_cast<float>(W));
  const float bh = std::max(8.0f, params_.box_h_ratio * static_cast<float>(H));

  auto make_box = [&](float cx, float cy) -> traffic::BBox {
    float x = cx - bw * 0.5f;
    float y = cy - bh * 0.5f;
    x = std::clamp(x, 0.0f, std::max(0.0f, static_cast<float>(W) - bw));
    y = std::clamp(y, 0.0f, std::max(0.0f, static_cast<float>(H) - bh));
    return traffic::BBox{x, y, bw, bh};
  };

  const int n = std::clamp(params_.num_boxes, 1, 2);
  const int64_t ts_ns = static_cast<int64_t>(traffic::Clock::nowNs());

  {
    traffic::Detection d{};
    d.ts_ns = ts_ns;
    d.bbox_img = make_box(0.5f * W, 0.55f * H);
    d.class_id = params_.class_id;
    d.score = params_.score;
    out.push_back(d);
  }

  if (n == 2) {
    traffic::Detection d{};
    d.ts_ns = ts_ns;
    d.bbox_img = make_box(0.65f * W, 0.45f * H);
    d.class_id = params_.class_id;
    d.score = std::max(0.0f, params_.score - 0.1f);
    out.push_back(d);
  }

  return out;
}

} // namespace traffic::detect
