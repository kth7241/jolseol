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
  // Config paths (있으면 로드, 없으면 기본값)
  opt.ipm_yaml_path = "configs/ipm_cam01.yaml";
  opt.zones_json_path = "configs/zones_cam01.json";

  traffic::app::Pipeline pipe(opt);
  const int ret = pipe.run();
  std::cout << "exit code: " << ret << std::endl;
  return ret;
}
