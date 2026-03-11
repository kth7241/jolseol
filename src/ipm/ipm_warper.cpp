/*
  ipm_warper.cpp
  - OpenCV 기반 IPM 변환 실행 구현
*/

#include "ipm/ipm_warper.hpp"

#include <opencv2/imgproc.hpp> // warpPerspective, INTER_LINEAR
#include <opencv2/core.hpp>    // perspectiveTransform

namespace traffic {

IpmWarper::IpmWarper(const IpmConfig& cfg) : cfg_(cfg) {
  // cfg.H(Matx33f)를 OpenCV Mat로 변환해 저장
  // - 내부 계산 안정성을 위해 double로 저장하는 편이 일반적으로 안전
  H_ = cv::Mat(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      H_.at<double>(r, c) = static_cast<double>(cfg_.H(r, c));
}

cv::Mat IpmWarper::warp(const cv::Mat& frame_bgr) const {
  // 입력이 비어있으면 그대로 empty 반환
  if (frame_bgr.empty()) return cv::Mat();

  cv::Mat ipm_bgr;
  // warpPerspective:
  // - 입력 img -> 출력 ipm
  // - 출력 크기: cfg_.ipm_size
  // - 보간: INTER_LINEAR(기본)
  // - border: 밖은 검정(기본 BORDER_CONSTANT)
  cv::warpPerspective(
      frame_bgr,
      ipm_bgr,
      H_,
      cfg_.ipm_size,
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT,
      cv::Scalar(0, 0, 0));

  return ipm_bgr;
}

cv::Point2f IpmWarper::warpPoint(const cv::Point2f& point_img) const {
  // perspectiveTransform은 vector 형태를 받는 경우가 많아서 1개짜리로 처리
  std::vector<cv::Point2f> src(1), dst;
  src[0] = point_img;

  cv::perspectiveTransform(src, dst, H_);
  return dst.empty() ? cv::Point2f(0.f, 0.f) : dst[0];
}

} // namespace traffic
