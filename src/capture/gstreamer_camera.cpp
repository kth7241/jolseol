#include "capture/gstreamer_camera.hpp"

#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "common/clock.hpp"
#include "common/logging.hpp"

namespace traffic {

struct GStreamerCamera::Impl {
  CameraConfig cfg;
  cv::VideoCapture cap;
  std::string pipeline_used;
  std::int64_t frame_id = 0;

  static void set_err(std::string* err, const std::string& msg) {
    if (err) *err = msg;
  }

  /**
   * @brief cfg 기반으로 기본 GStreamer pipeline 생성
   *
   * 출력 포맷:
   * - appsink로 BGR을 받도록 구성(후단에서 OpenCV cv::Mat으로 받기 쉬움)
   *
   * drop 정책:
   * - drop=1이면 내부 버퍼가 쌓일 때 오래된 프레임을 버려서 지연 누적을 막는다(실시간에 유리)
   */
  static std::string make_default_pipeline(const CameraConfig& cfg) {
    std::ostringstream ss;

    if (cfg.source == "csi") {
      // Jetson CSI 카메라 (nvarguscamerasrc)
      // - memory:NVMM → nvvidconv로 일반 메모리로 변환
      // - 최종 appsink에서 BGR로 받도록 구성
      ss << "nvarguscamerasrc ! "
         << "video/x-raw(memory:NVMM), width=" << cfg.width
         << ", height=" << cfg.height
         << ", framerate=" << cfg.fps << "/1 ! "
         << "nvvidconv ";
      if (cfg.flip) {
        // flip-method는 nvvidconv에서 지원(0~7), 여기서는 예시로 2(180도) 등 가능
        // 필요하면 cfg에 flip-method 숫자 옵션을 추가하는 게 더 좋다.
        ss << "flip-method=2 ";
      }
      ss << "! video/x-raw, format=BGRx ! "
         << "videoconvert ! video/x-raw, format=BGR ! "
         << "appsink";
      if (cfg.drop) ss << " drop=1";
    } else if (cfg.source == "v4l2") {
      // USB 카메라 (/dev/video0)
      // device는 반드시 지정 권장
      std::string dev = cfg.device.empty() ? "/dev/video0" : cfg.device;

      ss << "v4l2src device=" << dev << " ! "
         << "video/x-raw, width=" << cfg.width
         << ", height=" << cfg.height
         << ", framerate=" << cfg.fps << "/1 ! "
         << "videoconvert ! video/x-raw, format=BGR ! "
         << "appsink";
      if (cfg.drop) ss << " drop=1";
    } else {
      // file은 pipeline 대신 OpenCV의 파일 열기 사용 권장
      // (여기서는 빈 문자열 반환 -> 상위 open에서 file 처리)
      return "";
    }

    return ss.str();
  }
};

GStreamerCamera::GStreamerCamera() : impl_(new Impl) {}
GStreamerCamera::~GStreamerCamera() { close(); }

bool GStreamerCamera::open(const CameraConfig& cfg, std::string* err) {
  impl_->cfg = cfg;
  impl_->frame_id = 0;

  // 1) 파일 입력이면 pipeline 없이 VideoCapture(file) 사용
  if (cfg.source == "file") {
    if (cfg.device.empty()) {
      Impl::set_err(err, "CameraConfig.device is empty for source='file'. Provide video file path.");
      return false;
    }
    impl_->pipeline_used = cfg.device;
    impl_->cap.open(cfg.device);
    if (!impl_->cap.isOpened()) {
      Impl::set_err(err, "Failed to open video file: " + cfg.device);
      return false;
    }
    LOG_INFO("Opened file source: %s", cfg.device.c_str());
    return true;
  }

  // 2) gstreamer pipeline 결정
  std::string pipe = cfg.pipeline;
  if (pipe.empty()) {
    pipe = Impl::make_default_pipeline(cfg);
    if (pipe.empty()) {
      Impl::set_err(err, "Failed to make default pipeline. Check source/device settings.");
      return false;
    }
  }
  impl_->pipeline_used = pipe;

  // 3) OpenCV CAP_GSTREAMER로 오픈
  impl_->cap.open(pipe, cv::CAP_GSTREAMER);
  if (!impl_->cap.isOpened()) {
    Impl::set_err(err, "Failed to open GStreamer pipeline. Pipeline: " + pipe);
    return false;
  }

  LOG_INFO("Opened GStreamer pipeline: %s", pipe.c_str());
  return true;
}

bool GStreamerCamera::grab(FramePacket& out, std::string* err) {
  if (!impl_->cap.isOpened()) {
    Impl::set_err(err, "Camera is not opened.");
    return false;
  }

  cv::Mat frame;
  if (!impl_->cap.read(frame) || frame.empty()) {
    // 파일이면 EOF일 수 있음, 카메라면 장치 문제
    Impl::set_err(err, "Failed to read frame (EOF or camera error).");
    return false;
  }

  // FramePacket 채우기 (types.hpp 계약 기반)
  // - out.bgr: BGR image
  // - out.ts_ns: timestamp in ns (steady clock)
  // - out.frame_id: incremental id
  out.bgr = frame;
  out.ts_ns = traffic::nowNs();
  out.frame_id = impl_->frame_id++;

  return true;
}

void GStreamerCamera::close() {
  if (impl_ && impl_->cap.isOpened()) {
    impl_->cap.release();
  }
}

const std::string& GStreamerCamera::pipeline() const {
  return impl_->pipeline_used;
}

} // namespace traffic