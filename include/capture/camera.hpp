#pragma once
/**
 * @file camera.hpp
 * @brief 카메라 인터페이스(약결합). app/pipeline은 ICamera만 의존한다.
 *
 * MVP 목표:
 * - grab(FramePacket& out) 형태로 "한 프레임"을 제공.
 * - 구현체는 CSI-2(GStreamer) 또는 파일/USB 등으로 교체 가능.
 *
 * 약결합 포인트:
 * - pipeline은 구체 구현(GStreamerCamera)을 직접 사용하지 않고 ICamera만 사용.
 * - FramePacket 타입은 common/types.hpp 계약을 사용.
 *
 * FramePacket 필드 가정(프로젝트 설명 기반):
 * - out.bgr      : BGR 이미지(cv::Mat)
 * - out.frame_id : 프레임 번호(int64)
 * - out.ts_ns    : 타임스탬프(ns)
 *
 * ⚠️ 만약 types.hpp의 필드명이 다르면(예: ts_ns 대신 ts) 여기 구현부에서 컴파일 에러가 난다.
 *    그 경우 types.hpp와 동일한 이름으로 맞춰주면 됨(가급적 types.hpp의 계약대로 고정 권장).
 */

#include <string>
#include <cstdint>

#include "common/types.hpp"  // FramePacket 계약 사용

namespace traffic {

/**
 * @brief 카메라 설정.
 *
 * source:
 *  - "csi"  : nvarguscamerasrc 기반 CSI 카메라 (Jetson에서 흔함)
 *  - "v4l2" : /dev/videoX USB 카메라
 *  - "file" : 녹화 영상 파일 경로를 device에 넣어 사용
 *
 * pipeline:
 *  - 비어 있으면 구현체가 source/device/width/height/fps 기반으로 기본 파이프라인 생성
 *  - 사용자가 원하는 GStreamer 파이프라인이 있으면 pipeline에 직접 넣기
 *
 * 주의:
 * - Jetson CSI는 일반적으로 CAP_GSTREAMER로 접근할 때 nvarguscamerasrc를 사용
 */
struct CameraConfig {
  std::string name{"cam01"};
  std::string source{"csi"};     ///< "csi" / "v4l2" / "file"
  std::string device{""};        ///< v4l2면 "/dev/video0", file이면 "test.mp4" 등
  std::string pipeline{""};      ///< 사용자 지정 GStreamer pipeline (비어 있으면 자동 생성)

  int width{1920};
  int height{1080};
  int fps{30};

  bool drop{true};               ///< appsink drop=1 권장(지연 누적 방지)
  bool flip{false};              ///< 간단 플립 옵션(필요 시 구현에서 처리)
};

/**
 * @brief 카메라 인터페이스
 *
 * open():
 *  - 장치/파이프라인 오픈
 *
 * grab(out):
 *  - 프레임 1장을 out에 채움
 *  - 성공 true / 실패 false (EOF, 장치 에러 등)
 *
 * close():
 *  - 자원 해제
 */
class ICamera {
public:
  virtual ~ICamera() = default;

  virtual bool open(const CameraConfig& cfg, std::string* err = nullptr) = 0;
  virtual bool grab(FramePacket& out, std::string* err = nullptr) = 0;
  virtual void close() = 0;
};

} // namespace traffic