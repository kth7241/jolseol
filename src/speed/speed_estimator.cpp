// src/speed/speed_estimator.cpp
#include "speed/speed_estimator.hpp"

#include <algorithm>
#include <cmath>

#include "speed/filters.hpp"

namespace traffic {

SpeedEstimator::SpeedEstimator(const Options& opt) : opt_(opt) {
  if (opt_.ema_alpha < 0.f) opt_.ema_alpha = 0.f;
  if (opt_.ema_alpha > 1.f) opt_.ema_alpha = 1.f;
  if (opt_.dt_min <= 0.f) opt_.dt_min = 0.01f;
  if (opt_.dt_max < opt_.dt_min) opt_.dt_max = opt_.dt_min;
  if (opt_.max_speed_kmh <= 0.f) opt_.max_speed_kmh = 200.f;
  if (opt_.purge_timeout_sec < 0.f) opt_.purge_timeout_sec = 0.f;
  if (opt_.min_displacement_px < 0.f) opt_.min_displacement_px = 0.f;
}

std::vector<SpeedResult> SpeedEstimator::compute(const std::vector<TrackFoot>& feet,
                                                 float meters_per_pixel) {
  std::vector<SpeedResult> out;
  out.reserve(feet.size());

  // IPM scale 자체가 잘못되면 계산 불가
  if (!(meters_per_pixel > 0.f) || std::isnan(meters_per_pixel) || std::isinf(meters_per_pixel)) {
    for (const auto& f : feet) {
      SpeedResult s{};
      s.ts_ns = f.ts_ns;
      s.track_id = f.track_id;
      s.class_id = f.class_id;
      s.speed_kmh = 0.f;
      s.quality = SpeedQuality::kInvalidInput;
      s.zone_id = f.zone_id;
      s.lane_id = f.lane_id;
      out.push_back(s);
    }
    return out;
  }

  // 이번 배치 기준 최신 시각으로 stale state 정리
  int64_t now_ns = 0;
  for (const auto& f : feet) {
    now_ns = std::max(now_ns, static_cast<int64_t>(f.ts_ns));
  }
  const int64_t timeout_ns = static_cast<int64_t>(opt_.purge_timeout_sec * 1e9);
  if (now_ns > 0 && timeout_ns > 0) {
    states_.purge(now_ns, timeout_ns);
  }

  for (const auto& f : feet) {
    SpeedResult s{};
    s.ts_ns = f.ts_ns;
    s.track_id = f.track_id;
    s.class_id = f.class_id;
    s.zone_id = f.zone_id;
    s.lane_id = f.lane_id;

    if (f.track_id < 0 || f.ts_ns == kInvalidTs ||
        !std::isfinite(f.foot_ipm.x) || !std::isfinite(f.foot_ipm.y)) {
      s.speed_kmh = 0.f;
      s.quality = SpeedQuality::kInvalidInput;
      out.push_back(s);
      continue;
    }

    if (opt_.require_zone && !f.zone_id.has_value()) {
      s.speed_kmh = 0.f;
      s.quality = SpeedQuality::kNoZone;
      out.push_back(s);
      continue;
    }

    TrackState& st = states_.get(f.track_id);

    // 이전 프레임 정보가 없으면 이번 프레임을 기준점으로만 저장
    if (!st.initialized) {
      st.prev_foot_ipm = f.foot_ipm;
      st.prev_ts_ns = static_cast<int64_t>(f.ts_ns);
      st.prev_speed_kmh = 0.f;
      st.initialized = true;

      s.speed_kmh = 0.f;
      s.quality = SpeedQuality::kNoPrev;
      out.push_back(s);
      continue;
    }

    const double dt_raw = (static_cast<double>(f.ts_ns) - static_cast<double>(st.prev_ts_ns)) * 1e-9;
    if (!(dt_raw > 0.0)) {
      s.speed_kmh = st.prev_speed_kmh;
      s.quality = SpeedQuality::kBadDt;
      out.push_back(s);
      continue;
    }

    // dt가 너무 튀는 경우를 완전히 버리지는 않고, 안정적인 범위로 제한
    const double dt_s = std::clamp(dt_raw,
                                   static_cast<double>(opt_.dt_min),
                                   static_cast<double>(opt_.dt_max));

    const double dx = static_cast<double>(f.foot_ipm.x) - static_cast<double>(st.prev_foot_ipm.x);
    const double dy = static_cast<double>(f.foot_ipm.y) - static_cast<double>(st.prev_foot_ipm.y);
    double disp_px = std::sqrt(dx * dx + dy * dy);

    // bbox/footpoint 미세 흔들림은 실제 이동으로 보지 않음
    if (disp_px < static_cast<double>(opt_.min_displacement_px)) {
      disp_px = 0.0;
    }

    const double disp_m = disp_px * static_cast<double>(meters_per_pixel);
    double speed_kmh = (disp_m / dt_s) * 3.6;

    SpeedQuality q = SpeedQuality::kOk;
    if (!std::isfinite(speed_kmh)) {
      speed_kmh = 0.0;
      q = SpeedQuality::kInvalidInput;
    }

    if (speed_kmh > static_cast<double>(opt_.max_speed_kmh)) {
      speed_kmh = static_cast<double>(opt_.max_speed_kmh);
      q = SpeedQuality::kOutlierClamped;
    }

    const float smoothed = traffic::ema(st.prev_speed_kmh,
                                        static_cast<float>(speed_kmh),
                                        opt_.ema_alpha);

    // 다음 계산을 위해 현재 값을 저장
    st.prev_foot_ipm = f.foot_ipm;
    st.prev_ts_ns = static_cast<int64_t>(f.ts_ns);
    st.prev_speed_kmh = smoothed;

    s.speed_kmh = smoothed;
    s.quality = q;
    s.dt_s = static_cast<float>(dt_s);
    s.displacement_m = static_cast<float>(disp_m);

    out.push_back(s);
  }

  return out;
}

} // namespace traffic
