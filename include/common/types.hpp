// include/common/types.hpp
#pragma once
/**
 * @file types.hpp
 * @brief Project-wide shared data contracts (weak-coupling 핵심).
 *
 * ✅ 원칙
 * - 모듈 간 데이터 교환은 "이 파일 타입"만 사용한다.
 * - app/pipeline만이 모듈들을 결합한다.
 *
 * ✅ 좌표계/단위
 * - img 좌표: 원본 영상 픽셀 (x: right, y: down)
 * - ipm 좌표: top-view 영상 픽셀 (x: right, y: down)
 * - meters_per_pixel: ipm 픽셀 1칸이 실제 몇 m인지 [m/px]
 * - 속도 출력: km/h
 *
 * ✅ 시간
 * - TimestampNs: steady_clock 기반 ns(단조증가). 모듈 간 동기화/매칭에 사용.
 */

#include <cstdint>
#include <optional>
#include <limits>
#include <opencv2/core.hpp>

namespace traffic {

using FrameId     = std::uint64_t;
using TimestampNs = std::int64_t;

inline constexpr TimestampNs kInvalidTs = -1;

// 좌표 타입은 OpenCV 타입을 그대로 사용(호환성/연산 편의)
using Pt2f = cv::Point2f;

/**
 * @brief Axis-aligned bounding box in image pixel coordinates.
 * (x,y)=top-left, (w,h)>0.
 */
struct BBox {
  float x{0.f};
  float y{0.f};
  float w{0.f};
  float h{0.f};

  inline float x2() const { return x + w; }
  inline float y2() const { return y + h; }
  inline float cx() const { return x + 0.5f * w; }
  inline float cy() const { return y + 0.5f * h; }
  inline bool valid() const { return (w > 0.f) && (h > 0.f); }
};

/**
 * @brief Camera frame packet that flows through pipeline.
 *
 * NOTE: existing capture module fills these field names:
 * - out.bgr
 * - out.ts_ns
 * - out.frame_id
 */
struct FramePacket {
  FrameId frame_id{0};
  TimestampNs ts_ns{kInvalidTs};

  cv::Mat bgr;       // CV_8UC3 BGR (main frame)
  cv::Mat gray;      // optional gray
  cv::Mat small_bgr; // optional resized for detector

  int width{0};
  int height{0};

  inline bool valid() const { return ts_ns != kInvalidTs && !bgr.empty(); }
};

/**
 * @brief Detection output (YOLO etc.)
 */
struct Detection {
  TimestampNs ts_ns{kInvalidTs};
  BBox bbox_img;   // image px
  int class_id{-1};
  float score{0.f};
};

/**
 * @brief Tracker output (object with persistent id)
 */
struct Track {
  TimestampNs ts_ns{kInvalidTs};
  int track_id{-1};

  BBox bbox_img;   // image px
  int class_id{-1};
  float score{0.f};

  int age{0};
  int lost{0};
};

/**
 * @brief Track footpoint in image & IPM plane.
 */
struct TrackFoot {
  TimestampNs ts_ns{kInvalidTs};
  int track_id{-1};
  int class_id{-1};

  Pt2f foot_img;  // image px
  Pt2f foot_ipm;  // ipm px

  std::optional<int> zone_id;
  std::optional<int> lane_id;
};

enum class SpeedQuality : std::uint8_t {
  kOk = 0,
  kNoPrev,
  kBadDt,
  kOutlierClamped,
  kNoZone,
  kInvalidInput
};

struct SpeedResult {
  TimestampNs ts_ns{kInvalidTs};
  int track_id{-1};
  int class_id{-1};

  float speed_kmh{0.f};
  SpeedQuality quality{SpeedQuality::kOk};

  std::optional<int> zone_id;
  std::optional<int> lane_id;

  float dt_s{0.f};
  float displacement_m{0.f};
};

inline constexpr float kNaNf = std::numeric_limits<float>::quiet_NaN();

} // namespace traffic
