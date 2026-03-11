#pragma once
/*
  ipm_config.hpp
  - IPM(Inverse Perspective Mapping) 설정을 로드/검증하기 위한 최소 인터페이스.
  - 입력 파일: configs/ipm_cam01.yaml (OpenCV FileStorage YAML 포맷 지원)

  좌표계/단위:
  - img 좌표계: 원본 카메라 이미지 픽셀 좌표 (x: 오른쪽 +, y: 아래 +)
  - ipm 좌표계: Top-view(버드아이)로 변환된 이미지의 픽셀 좌표 (x: 오른쪽 +, y: 아래 +)
  - 실제 거리(m): meters_per_pixel(mpp)을 이용해 ipm 픽셀 이동량을 미터로 변환
      distance_m = distance_ipm_pixels * meters_per_pixel

  파일에 포함되어야 하는 키:
  - H: 3x3 homography (img -> ipm)
  - meters_per_pixel: float (>0)
  - ipm_size: [width, height] 또는 {width:..., height:...} 형태(양수)

  목표:
  - (MVP) 최소 설정 로드 + 유효성 검사 + ipm_warper에서 사용 가능
*/

#include <opencv2/core.hpp>
#include <string>

namespace traffic {

struct IpmConfig {
  // img -> ipm 변환 homography
  // - float 사용 (내부 계산은 double로 해도 됨)
  cv::Matx33f H = cv::Matx33f::eye();

  // ipm 픽셀 1칸이 실제 몇 미터인지 (m/pixel)
  float meters_per_pixel = 0.05f;

  // IPM 출력 영상 크기 (픽셀 단위)
  // - width, height 모두 양수여야 함
  cv::Size ipm_size = cv::Size(640, 480);
};

/*
  load_ipm_config
  - ipm_cam01.yaml에서 IpmConfig를 로드하고 유효성 검증까지 수행한다.

  Params:
  - yaml_path: YAML 파일 경로
  - out: 로드 결과 저장
  - err: 실패 시 에러 메시지(원인 설명). nullptr이면 메시지 생략.

  Returns:
  - true: 로드 및 검증 성공
  - false: 파일 열기 실패 / 키 누락 / 형식 오류 / 값 범위 오류 등
*/
bool load_ipm_config(const std::string& yaml_path, IpmConfig& out, std::string* err = nullptr);

/*
  validate_ipm_config
  - 이미 만들어진 cfg가 유효한지 검사.
  - 로더 외에, 툴/런타임에서 cfg를 수정한 뒤 확인할 때도 사용 가능.

  Checks (최소):
  - H 3x3 유효 (NaN/Inf 없음, det 너무 작지 않음)
  - meters_per_pixel > 0
  - ipm_size.width/height > 0
*/
bool validate_ipm_config(const IpmConfig& cfg, std::string* err = nullptr);

} // namespace traffic
