// traffic_monitor/include/zone/zone_manager.hpp
#pragma once
#include "zone/zone_config.hpp"
#include <opencv2/core.hpp>

namespace traffic {

/**
 * ZoneManager
 * - 단일 책임: "점이 어느 zone/lane에 속하는지" 판정
 * - 입력 점은 IPM 좌표계(pixel)라고 가정
 * - detect/track/speed 알고리즘 절대 포함 금지
 */
class ZoneManager {
public:
    ZoneManager() = default;

    // JSON 설정을 로드해서 내부에 보관한다.
    // 성공 시 true, 실패 시 false
    bool load(const std::string& zones_json_path);

    // 이미 로드된 config 접근(읽기 전용)
    const ZoneConfig& config() const { return cfg_; }

    /**
     * zoneOf
     * @param point_ipm IPM(pixel) 좌표계의 점
     * @return 포함되는 zone_id, 없으면 -1
     */
    int zoneOf(const cv::Point2f& point_ipm) const;

    /**
     * laneOf
     * @param point_ipm IPM(pixel) 좌표계의 점
     * @return 포함되는 lane_id, 없으면 -1
     * 정책: 해당 점이 어떤 zone polygon에 속하면 그 zone의 lane_id 반환
     */
    int laneOf(const cv::Point2f& point_ipm) const;

private:
    ZoneConfig cfg_;
};

} // namespace traffic
