#include "detect/onnx_yolo_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <filesystem>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "common/clock.hpp"
#include "common/logging.hpp"

namespace traffic::detect {

namespace {

int clampInt(int v, int lo, int hi) {
  return std::max(lo, std::min(v, hi));
}

} // namespace

OnnxYoloDetector::OnnxYoloDetector(const Options& opt) : opt_(opt) {
  if (opt_.model_path.empty()) {
    throw std::runtime_error("[OnnxYoloDetector] model_path is empty");
  }

  {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::exists(fs::path(opt_.model_path), ec)) {
      throw std::runtime_error(std::string("[OnnxYoloDetector] model file not found: ") + opt_.model_path);
    }
  }

  net_ = cv::dnn::readNetFromONNX(opt_.model_path);

  if (opt_.try_cuda) {
    try {
      // Jetson에서는 FP16 target이 더 유리한 경우가 많다.
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
      TM_LOGI("[detect] OnnxYoloDetector backend=CUDA_FP16 model=%s",
              opt_.model_path.c_str());
    } catch (...) {
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      TM_LOGW("[detect] CUDA backend setup failed. Fallback to CPU/OpenCV.");
    }
  } else {
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    TM_LOGI("[detect] OnnxYoloDetector backend=CPU model=%s",
            opt_.model_path.c_str());
  }
}

cv::Mat OnnxYoloDetector::preprocess_(const cv::Mat& frame, PreprocessInfo& info) const {
  const float scale_w = static_cast<float>(opt_.input_width) / static_cast<float>(frame.cols);
  const float scale_h = static_cast<float>(opt_.input_height) / static_cast<float>(frame.rows);
  info.scale = std::min(scale_w, scale_h);

  info.resized_w = static_cast<int>(std::round(frame.cols * info.scale));
  info.resized_h = static_cast<int>(std::round(frame.rows * info.scale));
  info.pad_x = (opt_.input_width - info.resized_w) / 2;
  info.pad_y = (opt_.input_height - info.resized_h) / 2;

  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(info.resized_w, info.resized_h));

  // YOLO letterbox 관례에 맞춰 회색 배경(114) 사용
  cv::Mat canvas(opt_.input_height, opt_.input_width, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect(info.pad_x, info.pad_y, info.resized_w, info.resized_h)));

  return cv::dnn::blobFromImage(
      canvas,
      1.0 / 255.0,
      cv::Size(opt_.input_width, opt_.input_height),
      cv::Scalar(),
      true,   // BGR -> RGB
      false,
      CV_32F);
}

std::string OnnxYoloDetector::shapeToString_(const cv::Mat& mat) {
  std::ostringstream oss;
  oss << "dims=" << mat.dims << " [";
  for (int i = 0; i < mat.dims; ++i) {
    oss << mat.size[i];
    if (i + 1 < mat.dims) oss << ", ";
  }
  oss << "]";
  return oss.str();
}

bool OnnxYoloDetector::isVehicleClass_(int class_id) {
  // COCO 기준: car(2), motorcycle(3), bus(5), truck(7)
  return class_id == 2 || class_id == 3 || class_id == 5 || class_id == 7;
}

std::vector<traffic::Detection> OnnxYoloDetector::postprocess_(
    const cv::Mat& frame,
    const std::vector<cv::Mat>& outputs,
    const PreprocessInfo& info) {
  std::vector<traffic::Detection> results;
  if (outputs.empty()) return results;

  const cv::Mat& out = outputs[0];
  if (opt_.debug_shape && !printed_shape_) {
    printed_shape_ = true;
    TM_LOGI("[detect] YOLO output shape: %s", shapeToString_(out).c_str());
  }

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> class_ids;

  auto decode_row = [&](const float* row_ptr, int num_values) {
    if (num_values < 6) return;

    const float cx = row_ptr[0];
    const float cy = row_ptr[1];
    const float w  = row_ptr[2];
    const float h  = row_ptr[3];

    int best_class_id = -1;
    float best_class_score = 0.f;
    for (int c = 4; c < num_values; ++c) {
      const float score = row_ptr[c];
      if (score > best_class_score) {
        best_class_score = score;
        best_class_id = c - 4;
      }
    }

    if (best_class_id < 0) return;
    if (!isVehicleClass_(best_class_id)) return;
    if (best_class_score < opt_.conf_threshold) return;

    const float x1_net = cx - 0.5f * w;
    const float y1_net = cy - 0.5f * h;
    const float x2_net = cx + 0.5f * w;
    const float y2_net = cy + 0.5f * h;

    // letterbox 역변환: pad 제거 후 scale 복원
    const float x1 = (x1_net - static_cast<float>(info.pad_x)) / info.scale;
    const float y1 = (y1_net - static_cast<float>(info.pad_y)) / info.scale;
    const float x2 = (x2_net - static_cast<float>(info.pad_x)) / info.scale;
    const float y2 = (y2_net - static_cast<float>(info.pad_y)) / info.scale;

    const int left   = clampInt(static_cast<int>(std::round(x1)), 0, frame.cols - 1);
    const int top    = clampInt(static_cast<int>(std::round(y1)), 0, frame.rows - 1);
    const int right  = clampInt(static_cast<int>(std::round(x2)), 0, frame.cols - 1);
    const int bottom = clampInt(static_cast<int>(std::round(y2)), 0, frame.rows - 1);

    const int bw = std::max(0, right - left);
    const int bh = std::max(0, bottom - top);
    if (bw <= 1 || bh <= 1) return;

    boxes.emplace_back(left, top, bw, bh);
    confidences.push_back(best_class_score);
    class_ids.push_back(best_class_id);
  };

  if (out.dims == 3 && out.size[0] == 1) {
    // YOLOv8 ONNX에서 자주 보이는 두 가지 shape 처리
    // 1) [1, 84, 8400]
    // 2) [1, 8400, 84]
    if (out.size[1] < out.size[2]) {
      // [1, C, N] -> transpose 해서 row 단위로 읽기
      const int channels = out.size[1];
      const int num_boxes = out.size[2];
      cv::Mat reshaped(channels, num_boxes, CV_32F, const_cast<float*>(out.ptr<float>()));
      cv::Mat transposed = reshaped.t();
      for (int i = 0; i < transposed.rows; ++i) {
        decode_row(transposed.ptr<float>(i), transposed.cols);
      }
    } else {
      // [1, N, C]
      const int num_boxes = out.size[1];
      const int channels = out.size[2];
      cv::Mat rows(num_boxes, channels, CV_32F, const_cast<float*>(out.ptr<float>()));
      for (int i = 0; i < rows.rows; ++i) {
        decode_row(rows.ptr<float>(i), rows.cols);
      }
    }
  } else {
    TM_LOGE("[detect] Unsupported YOLO output shape: %s", shapeToString_(out).c_str());
    return results;
  }

  std::vector<int> keep_indices;
  cv::dnn::NMSBoxes(boxes, confidences, opt_.conf_threshold, opt_.nms_threshold, keep_indices);

  const traffic::TimestampNs ts_ns = static_cast<traffic::TimestampNs>(traffic::Clock::nowNs());
  results.reserve(keep_indices.size());
  for (int idx : keep_indices) {
    const cv::Rect& b = boxes[idx];
    traffic::Detection det;
    det.ts_ns = ts_ns;
    det.bbox_img = traffic::BBox{static_cast<float>(b.x),
                                 static_cast<float>(b.y),
                                 static_cast<float>(b.width),
                                 static_cast<float>(b.height)};
    det.class_id = class_ids[idx];
    det.score = confidences[idx];
    results.push_back(det);
  }

  return results;
}

std::vector<traffic::Detection> OnnxYoloDetector::infer(const cv::Mat& frame_bgr) {
  if (frame_bgr.empty()) return {};

  PreprocessInfo info;
  cv::Mat blob = preprocess_(frame_bgr, info);

  net_.setInput(blob);
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());

  return postprocess_(frame_bgr, outputs, info);
}

} // namespace traffic::detect
