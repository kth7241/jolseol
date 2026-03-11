// src/main.cpp
#include "app/pipeline.hpp"

#include <iostream>

int main(int argc, char** argv) {
  (void)argc; (void)argv;

  traffic::app::Pipeline::Options opt;

  // NOTE:
  // - MVP default는 videotestsrc로 되어있습니다.
  // - Jetson CSI 카메라를 쓰려면 opt.cam_pipeline에 자신의 GStreamer pipeline 문자열을 넣으세요.
  //
  // 예시(프로젝트 환경에 맞게 수정 필요):
  opt.cam_pipeline =
  "nvarguscamerasrc sensor-id=0 ! "
  "video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1, format=NV12 ! "
  "nvvidconv flip-method=0 ! "
  "video/x-raw, format=BGRx ! "
  "videoconvert ! "
  "video/x-raw, format=BGR ! "
  "appsink drop=true sync=false max-buffers=1";
  // YOLO ONNX model path
  // - models/yolov8n.onnx 를 기본값으로 사용
  // - Jetson으로 옮긴 뒤에도 동일한 상대경로를 유지하면 편합니다.
  opt.yolo_onnx_path = "models/yolov8n.onnx";

  // Config paths
  // - 현재 통합본은 YOLO 검출이 우선이므로, 기본값은 비워둡니다.
  // - 실제 IPM / zone 설정 파일이 준비되면 그때 경로를 넣어서 사용하세요.
  opt.ipm_yaml_path = "";
  opt.zones_json_path = "";

  traffic::app::Pipeline pipe(opt);
  const int ret = pipe.run();
  std::cout << "exit code: " << ret << std::endl;
  return ret;
}
