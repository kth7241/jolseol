// include/detect/trt_yolo_detector.hpp
#pragma once
/**
 * @file trt_yolo_detector.hpp
 * @brief TensorRT YOLO detector wrapper (skeleton).
 *
 * MVP 단계:
 * - 엔진이 없어도 "컴파일/링크"가 되도록 뼈대만 제공한다.
 * - 실제 TensorRT 의존 코드는 Impl 내부 TODO로 남긴다.
 */

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "common/types.hpp"
#include "detect/detector.hpp"

namespace traffic::detect {

struct TrtYoloConfig {
  std::string engine_path;   // .engine path
  int input_w = 640;
  int input_h = 640;
  float conf_thresh = 0.25f;
  float iou_thresh = 0.45f;
  int max_det = 100;
};

class TrtYoloDetector final : public IDetector {
public:
  explicit TrtYoloDetector(const TrtYoloConfig& cfg);
  ~TrtYoloDetector() override;

  std::vector<traffic::Detection> infer(const cv::Mat& frame_bgr) override;

  bool ready() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace traffic::detect
