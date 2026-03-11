#pragma once
/**
 * @file onnx_yolo_detector.hpp
 * @brief OpenCV DNN 기반 YOLO ONNX 검출기
 *
 * 사용 목적:
 * - 기존 프로젝트의 detector stub 대신 실제 YOLO ONNX 모델을 연결한다.
 * - traffic::detect::IDetector 인터페이스를 그대로 따르므로,
 *   pipeline 쪽 수정은 최소화한다.
 */

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

#include "common/types.hpp"
#include "detect/detector.hpp"

namespace traffic::detect {

class OnnxYoloDetector final : public IDetector {
public:
  struct Options {
    std::string model_path;          ///< ONNX 모델 경로
    int input_width{640};            ///< 모델 입력 가로 크기
    int input_height{640};           ///< 모델 입력 세로 크기
    float conf_threshold{0.30f};     ///< confidence 임계값
    float nms_threshold{0.45f};      ///< NMS 임계값
    bool try_cuda{true};             ///< CUDA backend 시도 여부
    bool debug_shape{true};          ///< 첫 추론 때 출력 shape 로그 출력 여부
  };

  explicit OnnxYoloDetector(const Options& opt);
  ~OnnxYoloDetector() override = default;

  std::vector<traffic::Detection> infer(const cv::Mat& frame_bgr) override;

private:
  struct PreprocessInfo {
    float scale{1.0f};
    int pad_x{0};
    int pad_y{0};
    int resized_w{0};
    int resized_h{0};
  };

  Options opt_{};
  cv::dnn::Net net_;
  bool printed_shape_{false};

  cv::Mat preprocess_(const cv::Mat& frame, PreprocessInfo& info) const;
  std::vector<traffic::Detection> postprocess_(const cv::Mat& frame,
                                               const std::vector<cv::Mat>& outputs,
                                               const PreprocessInfo& info);

  static std::string shapeToString_(const cv::Mat& mat);
  static bool isVehicleClass_(int class_id);
};

} // namespace traffic::detect
