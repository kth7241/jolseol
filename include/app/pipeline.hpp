// include/app/pipeline.hpp
#pragma once
/**
 * @file pipeline.hpp
 * @brief MVP pipeline wiring (single-thread loop).
 *
 * MVP 목표:
 * - Camera -> Detector(Stub) -> IOU Tracker -> FootpointMapper -> IPM -> Zone -> Speed -> Overlay -> imshow
 *
 * 주의:
 * - 멀티스레드/큐는 MVP 안정화 이후에 추가한다.
 * - 여기서는 "일단 실행되는 것"이 우선.
 */

#include <memory>
#include <string>

namespace traffic {
  struct IpmConfig;
  struct ZoneConfig;
}

namespace traffic::app {

class Pipeline {
public:
  struct Options {
    // Paths (relative or absolute)
    std::string cam_pipeline;     // GStreamer pipeline string (if empty, use default)
    std::string zones_json_path;  // e.g., configs/zones_cam01.json
    std::string ipm_yaml_path;    // e.g., configs/ipm_cam01.yaml

    // Detector (YOLO ONNX)
    std::string yolo_onnx_path;   // e.g., models/yolov8n.onnx
    int yolo_input_w = 640;
    int yolo_input_h = 640;
    float yolo_conf_thresh = 0.30f;
    float yolo_nms_thresh = 0.45f;
    bool yolo_try_cuda = true;
    bool yolo_debug_shape = true;

    // Tracker params
    float iou_threshold = 0.3f;
    int max_age = 10;

    // Speed params
    float ema_alpha = 0.3f;
    float dt_min = 0.01f;
    float dt_max = 0.2f;
    float max_speed_kmh = 200.f;
    float purge_timeout_sec = 1.0f;

    // Display
    std::string window_name = "traffic_monitor_mvp";
  };

  explicit Pipeline(const Options& opt);
  ~Pipeline();

  // Build all modules and start running loop. Returns when user quits or error occurs.
  int run();

private:
  Options opt_;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace traffic::app
