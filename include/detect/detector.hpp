// include/detect/detector.hpp
#pragma once
/**
 * @file detector.hpp
 * @brief Detector interface + stub detector for MVP.
 */

#include <memory>
#include <vector>
#include <opencv2/core.hpp>

#include "common/types.hpp"

namespace traffic::detect {

class IDetector {
public:
  virtual ~IDetector() = default;
  virtual std::vector<traffic::Detection> infer(const cv::Mat& frame_bgr) = 0;
};

class DetectorStub final : public IDetector {
public:
  struct Params {
    int num_boxes{2};
    int class_id{0};
    float score{0.9f};
    float box_w_ratio{0.25f};
    float box_h_ratio{0.20f};
  };

  DetectorStub() = default;
  explicit DetectorStub(const Params& p) : params_(p) {}

  std::vector<traffic::Detection> infer(const cv::Mat& frame_bgr) override;

private:
  Params params_{};
};

std::unique_ptr<IDetector> CreateDetectorStub();

} // namespace traffic::detect
