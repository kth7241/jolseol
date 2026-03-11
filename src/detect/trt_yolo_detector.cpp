#include "detect/trt_yolo_detector.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>

#include "common/logging.hpp"
#include "common/clock.hpp"

namespace traffic::detect {

// ===========================
// PImpl 실제 구현(현재는 뼈대)
// ===========================
struct TrtYoloDetector::Impl {
  TrtYoloConfig cfg;
  bool ready = false;

  // TODO(실구현): 아래 멤버들은 TensorRT 타입을 사용하게 됨.
  // - nvinfer1::IRuntime*
  // - nvinfer1::ICudaEngine*
  // - nvinfer1::IExecutionContext*
  // - CUDA stream / device buffers
  //
  // 지금은 MVP 컴파일/링크를 위해 실제 타입을 넣지 않는다.

  explicit Impl(const TrtYoloConfig& c) : cfg(c) {}

  bool loadEngineIfPossible() {
    // TODO(실구현):
    // 1) cfg.engine_path 존재 여부 확인
    // 2) TensorRT runtime 생성, engine deserialize
    // 3) execution context 생성
    // 4) 입력/출력 binding 확인
    // 5) CUDA 버퍼 할당
    //
    // MVP 단계에서는 엔진이 없으니 ready=false 유지.
    return false;
  }

  std::vector<Detection> runInfer(const cv::Mat& frame) {
    (void)frame;

    // TODO(실구현):
    // 1) preprocess: resize/letterbox, normalize, HWC->CHW, FP16/FP32
    // 2) enqueueV2/executeV2 실행
    // 3) decode: YOLO output -> bbox/class/score
    // 4) NMS 적용
    // 5) bbox를 원본 프레임 픽셀 좌표로 역스케일
    //
    // 현재는 빈 결과 반환(엔진 미연결 시 동작)
    return {};
  }
};

TrtYoloDetector::TrtYoloDetector(const TrtYoloConfig& cfg)
  : impl_(std::make_unique<Impl>(cfg))
{
  try {
    impl_->ready = impl_->loadEngineIfPossible();
    if (!impl_->ready) {
      TM_LOG_WARN("[detect] TrtYoloDetector not ready (engine not loaded). "
                  "infer() will return empty results. engine_path=%s",
                  cfg.engine_path.c_str());
    } else {
      TM_LOG_INFO("[detect] TrtYoloDetector ready. engine_path=%s",
                  cfg.engine_path.c_str());
    }
  } catch (const std::exception& e) {
    impl_->ready = false;
    TM_LOG_ERROR("[detect] TrtYoloDetector init failed: %s", e.what());
  }
}

TrtYoloDetector::~TrtYoloDetector() = default;

bool TrtYoloDetector::isReady() const noexcept {
  return impl_ && impl_->ready;
}

std::vector<Detection> TrtYoloDetector::infer(const cv::Mat& frame) {
  if (!impl_) return {};
  if (!impl_->ready) return {};
  if (frame.empty()) return {};

  try {
    return impl_->runInfer(frame);
  } catch (const std::exception& e) {
    TM_LOG_ERROR("[detect] TrtYoloDetector::infer exception: %s", e.what());
    return {};
  }
}

} // namespace traffic::detect
