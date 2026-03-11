// traffic_monitor/include/zone/zone_config.hpp
#pragma once
/**
 * @file zone_config.hpp
 * @brief Load/validate lane zones from JSON.
 *
 * 책임:
 * - zones_cam01.json을 파싱해 Zone 리스트를 구성한다.
 * - 좌표계(coord_sys_) 문자열을 보관한다(권장: "ipm").
 *
 * 약결합:
 * - 이 파일은 detect/speed/ui 등과 무관해야 한다.
 * - OpenCV(cv::Point2f) + STL만 사용.
 */

#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace traffic {

struct ZonePoly {
  int zone_id{-1};
  int lane_id{-1};
  std::vector<cv::Point2f> poly;   // polygon vertices in configured coordinate system
};

class ZoneConfig {
public:
  bool loadFromJson(const std::string& path);

  const std::vector<ZonePoly>& zones() const { return zones_; }
  const std::string& coordinateSystem() const { return coord_sys_; }

private:
  std::string coord_sys_;           // e.g. "ipm" or "img"
  std::vector<ZonePoly> zones_;
  bool validate_() const;
};

} // namespace traffic
