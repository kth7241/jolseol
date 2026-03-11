/*
  ipm_config.cpp
  - OpenCV FileStorage 기반 YAML 로드 구현.
  - 외부 yaml-cpp로 교체하고 싶으면 이 cpp만 바꾸면 됨(헤더 인터페이스 유지).
*/

#include "ipm/ipm_config.hpp"
#include <opencv2/core.hpp>
#include <cmath>      // std::isfinite
#include <sstream>    // std::ostringstream

namespace traffic {

static void set_err(std::string* err, const std::string& msg) {
  if (err) *err = msg;
}

static bool mat_has_nan_inf(const cv::Mat& m) {
  // m이 float/double일 때 NaN/Inf 검사 (최소 구현)
  CV_Assert(m.depth() == CV_32F || m.depth() == CV_64F);
  const int rows = m.rows;
  const int cols = m.cols;

  if (m.depth() == CV_32F) {
    for (int r = 0; r < rows; ++r) {
      const float* p = m.ptr<float>(r);
      for (int c = 0; c < cols; ++c) {
        if (!std::isfinite(p[c])) return true;
      }
    }
  } else {
    for (int r = 0; r < rows; ++r) {
      const double* p = m.ptr<double>(r);
      for (int c = 0; c < cols; ++c) {
        if (!std::isfinite(p[c])) return true;
      }
    }
  }
  return false;
}

bool validate_ipm_config(const IpmConfig& cfg, std::string* err) {
  // meters_per_pixel
  if (!(cfg.meters_per_pixel > 0.0f) || !std::isfinite(cfg.meters_per_pixel)) {
    set_err(err, "meters_per_pixel must be finite and > 0.");
    return false;
  }

  // ipm_size
  if (cfg.ipm_size.width <= 0 || cfg.ipm_size.height <= 0) {
    set_err(err, "ipm_size must have positive width/height.");
    return false;
  }

  // H 유효성: det 너무 작으면(거의 특이행렬) 변환이 불안정해질 수 있음
  // Matx33f -> Mat로 변환 후 det 계산
  cv::Mat Hm(3, 3, CV_32F);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      Hm.at<float>(r, c) = cfg.H(r, c);

  if (mat_has_nan_inf(Hm)) {
    set_err(err, "H contains NaN or Inf.");
    return false;
  }

  // det는 스케일에 따라 달라질 수 있어 "0에 너무 가깝지 않은지"만 완화해서 체크
  const double det = cv::determinant(Hm);
  if (!std::isfinite(det) || std::abs(det) < 1e-12) {
    set_err(err, "H determinant is too small or not finite (singular/unstable).");
    return false;
  }

  return true;
}

bool load_ipm_config(const std::string& yaml_path, IpmConfig& out, std::string* err) {
  // OpenCV FileStorage로 YAML 로드
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    set_err(err, "Failed to open YAML: " + yaml_path);
    return false;
  }

  // 1) H 로드
  // 기대 형식 예시:
  // H: !!opencv-matrix
  //   rows: 3
  //   cols: 3
  //   dt: f
  //   data: [ ... 9개 ... ]
  cv::FileNode nH = fs["H"];
  if (nH.empty()) {
    set_err(err, "Missing key 'H' in YAML.");
    return false;
  }

  cv::Mat Hmat;
  nH >> Hmat;

  if (Hmat.empty() || Hmat.rows != 3 || Hmat.cols != 3) {
    set_err(err, "H must be a 3x3 matrix.");
    return false;
  }

  // float로 통일(입력이 double이어도 OK)
  Hmat.convertTo(Hmat, CV_32F);

  IpmConfig cfg;

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      cfg.H(r, c) = Hmat.at<float>(r, c);

  // 2) meters_per_pixel 로드
  cv::FileNode nmpp = fs["meters_per_pixel"];
  if (nmpp.empty()) {
    set_err(err, "Missing key 'meters_per_pixel' in YAML.");
    return false;
  }
  cfg.meters_per_pixel = (float)nmpp.real();

  // 3) ipm_size 로드 (두 형태 모두 허용)
  // 형태 A) ipm_size: [640, 480]
  // 형태 B) ipm_size: { width: 640, height: 480 }
  cv::FileNode nsize = fs["ipm_size"];
  if (nsize.empty()) {
    set_err(err, "Missing key 'ipm_size' in YAML.");
    return false;
  }

  int w = 0, h = 0;
  if (nsize.isSeq() && nsize.size() >= 2) {
    w = (int)nsize[0];
    h = (int)nsize[1];
  } else if (nsize.isMap()) {
    cv::FileNode nw = nsize["width"];
    cv::FileNode nh = nsize["height"];
    if (nw.empty() || nh.empty()) {
      set_err(err, "ipm_size map must contain width and height.");
      return false;
    }
    w = (int)nw;
    h = (int)nh;
  } else {
    set_err(err, "ipm_size must be a sequence [w,h] or map {width,height}.");
    return false;
  }
  cfg.ipm_size = cv::Size(w, h);

  // 최종 검증
  std::string vmsg;
  if (!validate_ipm_config(cfg, &vmsg)) {
    set_err(err, "Invalid IPM config: " + vmsg);
    return false;
  }

  out = cfg;
  return true;
}

} // namespace traffic
