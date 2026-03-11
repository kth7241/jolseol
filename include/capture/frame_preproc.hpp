#pragma once
/**
 * @file frame_preproc.hpp
 * @brief 프레임 전처리(가벼운 것만): resize, BGR->GRAY 생성 등
 *
 * 원칙:
 * - 여기서는 lane/ipm/yolo 같은 고수준 로직 금지
 * - "입출력 프레임 형태"만 정리해 다음 단계가 쓰기 쉽게 만든다.
 *
 * 좌표계:
 * - 입력/출력 모두 이미지 픽셀 좌표(원본 또는 리사이즈된 이미지)
 *
 * MVP에서 권장 사용:
 * - YOLO 입력이 640x640 같은 고정이면 여기서 small frame을 만들거나
 * - 혹은 detector 내부에서 처리해도 됨(팀 규칙에 따라 선택).
 */

#include <opencv2/core.hpp>

namespace traffic {

struct FramePreprocConfig {
  bool enable_resize{false};   ///< true면 bgr_small을 생성
  int out_w{1280};             ///< resize width
  int out_h{720};              ///< resize height

  bool make_gray{false};       ///< true면 gray를 생성
};

class FramePreproc {
public:
  explicit FramePreproc(const FramePreprocConfig& cfg);

  /**
   * @brief 전처리 실행
   *
   * @param bgr_in  입력 BGR (원본)
   * @param bgr_out 출력 BGR
   *               - enable_resize==true면 resized 결과
   *               - 아니면 bgr_in을 그대로 복사 참조(out = in) or clone 정책은 구현 선택
   * @param gray_out (optional) GRAY 출력 포인터
   *                 - make_gray==true면 유효한 Mat로 채움
   *                 - nullptr이면 생성하지 않음
   *
   * 예외/주의:
   * - 입력이 empty면 출력도 empty
   */
  void run(const cv::Mat& bgr_in, cv::Mat& bgr_out, cv::Mat* gray_out = nullptr) const;

  const FramePreprocConfig& config() const { return cfg_; }

private:
  FramePreprocConfig cfg_;
};

} // namespace traffic