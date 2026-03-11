// traffic_monitor/src/zone/zone_manager.cpp
#include "zone/zone_manager.hpp"
#include "common/logging.hpp"

// geometry/polygon.cpp를 선택 구현했다면 이걸 사용.
// 여기서는 ray casting 기반 point-in-polygon 유틸을 직접 사용한다.
#include "geometry/polygon.hpp"

namespace traffic {

bool ZoneManager::load(const std::string& zones_json_path) {
    if (!cfg_.loadFromJson(zones_json_path)) {
        TM_LOGE("ZoneManager: failed to load zones: %s", zones_json_path.c_str());
        return false;
    }

    // 현재 프로젝트는 zone polygon 좌표계를 IPM으로 쓰는 것을 기본 전제로 한다.
    if (!cfg_.coordinateSystem().empty() && cfg_.coordinateSystem() != "ipm") {
        TM_LOGW("ZoneManager: coordinate_system=%s (expected ipm).",
                cfg_.coordinateSystem().c_str());
    }
    return true;
}

int ZoneManager::zoneOf(const cv::Point2f& point_ipm) const {
    // 정책:
    // - zone 목록 순서대로 검사
    // - 가장 먼저 포함되는 zone을 반환
    // - 아무 곳에도 속하지 않으면 -1 반환
    for (const auto& z : cfg_.zones()) {
        if (traffic::geom::pointInPolygonRayCasting(z.poly, point_ipm, true)) {
            return z.zone_id;
        }
    }
    return -1;
}

int ZoneManager::laneOf(const cv::Point2f& point_ipm) const {
    // lane 역시 같은 polygon 기준으로 판정한다.
    // 즉 "어느 zone 안에 있느냐"를 먼저 보고,
    // 그 zone에 연결된 lane_id를 반환한다.
    for (const auto& z : cfg_.zones()) {
        if (traffic::geom::pointInPolygonRayCasting(z.poly, point_ipm, true)) {
            return z.lane_id;
        }
    }
    return -1;
}

} // namespace traffic
