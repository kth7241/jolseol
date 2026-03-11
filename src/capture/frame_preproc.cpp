#include "capture/frame_preproc.hpp"

#include <opencv2/imgproc.hpp>

namespace traffic {

FramePreproc::FramePreproc(const FramePreprocConfig& cfg) : cfg_(cfg) {}

void FramePreproc::run(const cv::Mat& bgr_in, cv::Mat& bgr_out, cv::Mat* gray_out) const {
  if (bgr_in.empty()) {
    bgr_out.release();
    if (gray_out) gray_out->release();
    return;
  }

  // 1) Resize 여부
  if (cfg_.enable_resize) {
    cv::resize(bgr_in, bgr_out, cv::Size(cfg_.out_w, cfg_.out_h), 0, 0, cv::INTER_LINEAR);
  } else {
    // 성능 우선: 얕은 복사(참조 공유)
    // - 이후 단계에서 bgr_out을 수정한다면 clone() 필요
    bgr_out = bgr_in;
  }

  // 2) Gray 생성 여부
  if (gray_out) {
    if (cfg_.make_gray) {
      cv::cvtColor(bgr_out, *gray_out, cv::COLOR_BGR2GRAY);
    } else {
      gray_out->release();
    }
  }
}

} // namespace traffic