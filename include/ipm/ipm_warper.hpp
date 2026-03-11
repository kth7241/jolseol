#pragma once
/*
  ipm_warper.hpp
  - IPM 변환 실행 모듈(최소 구현)
  - OpenCV warpPerspective / perspectiveTransform 사용

  핵심:
  - 입력 frame_bgr(img 좌표계, 픽셀) -> 출력 ipm_frame_bgr(ipm 좌표계, 픽셀)
  - point_img(img 픽셀) -> point_ipm(ipm 픽셀)
  - 실제 거리 변환은 meters_per_pixel을 호출자가 사용

  의존:
  - OpenCV만 사용
*/

#include "ipm/ipm_config.hpp"
#include <opencv2/core.hpp>

namespace traffic {

class IpmWarper {
public:
  /*
    생성자
    - cfg.H: img -> ipm homography
    - cfg.ipm_size: 출력 IPM 이미지 크기
    - cfg.meters_per_pixel: 거리 변환에 사용
  */
  explicit IpmWarper(const IpmConfig& cfg);

  /*
    warp
    - 입력: frame_bgr (원본 img 좌표계, BGR)
    - 출력: ipm_frame_bgr (top-view ipm 좌표계, BGR)

    예외/주의:
    - frame_bgr가 empty이면 empty Mat 반환
    - 출력 크기는 cfg.ipm_size로 고정
  */
  cv::Mat warp(const cv::Mat& frame_bgr) const;

  /*
    warpPoint
    - 입력: point_img (원본 이미지 픽셀 좌표)
    - 출력: point_ipm (IPM 이미지 픽셀 좌표)

    사용처:
    - track bbox -> foot_img 계산 후, 이 함수로 foot_ipm 계산할 때 사용

    예외/주의:
    - 내부적으로 homography를 적용하며, 점이 변환 가능한 범위 밖이면 값이 화면 밖으로 나갈 수 있음.
      (클리핑/범위 판정은 zone/speed 등 상위 단계에서 처리 권장)
  */
  cv::Point2f warpPoint(const cv::Point2f& point_img) const;

  // meters_per_pixel getter (m/pixel)
  float metersPerPixel() const { return cfg_.meters_per_pixel; }

  // IPM 출력 영상 크기 getter (pixels)
  cv::Size ipmSize() const { return cfg_.ipm_size; }

  // raw config 접근(디버그/로그용)
  const IpmConfig& config() const { return cfg_; }

private:
  IpmConfig cfg_;

  // OpenCV 함수들이 Mat 형태를 편하게 쓰므로, H를 Mat로도 보관
  cv::Mat H_; // 3x3 CV_64F or CV_32F (구현에서 설정)
};

  // Compatibility alias (older code may refer to IPMWarper)
  using IPMWarper = IpmWarper;

} // namespace traffic
