#pragma once
/**
 * @file gstreamer_camera.hpp
 * @brief OpenCV VideoCapture(CAP_GSTREAMER) 기반 카메라 구현체.
 *
 * 목적:
 * - MVP에서 Jetson Orin Nano CSI 카메라를 빠르게 붙이기 위한 구현체.
 * - 나중에 Argus 직접 API, V4L2 직접 제어로 바꾸더라도 ICamera 인터페이스는 유지.
 *
 * 의존성:
 * - 헤더는 "OpenCV 전체 include"를 피하기 위해 PIMPL 사용.
 * - 실제 OpenCV VideoCapture include는 .cpp에서만 한다.
 */

#include <memory>
#include <string>

#include "capture/camera.hpp"

namespace traffic {

class GStreamerCamera final : public ICamera {
public:
  GStreamerCamera();
  ~GStreamerCamera() override;

  bool open(const CameraConfig& cfg, std::string* err = nullptr) override;
  bool grab(FramePacket& out, std::string* err = nullptr) override;
  void close() override;

  /// @brief 실제 사용 중인 파이프라인 문자열(디버그용)
  const std::string& pipeline() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace traffic