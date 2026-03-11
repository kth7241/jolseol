// traffic_monitor/src/zone/zone_config.cpp
#include "zone/zone_config.hpp"
#include "common/logging.hpp"

#include <fstream>
#include <sstream>
#include <unordered_set>

// ---- JSON 파서: nlohmann/json 사용 (있으면 실제 파싱)
// 없으면 컴파일 가능한 Stub 경로로 false 반환 ----
#if __has_include(<nlohmann/json.hpp>)
  #include <nlohmann/json.hpp>
  #define TM_HAS_NLOHMANN_JSON 1
#else
  #define TM_HAS_NLOHMANN_JSON 0
#endif

namespace traffic {

static bool readTextFile_(const std::string& path, std::string& out) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;
    std::ostringstream ss;
    ss << ifs.rdbuf();
    out = ss.str();
    return true;
}

bool ZoneConfig::loadFromJson(const std::string& path) {
    zones_.clear();
    coord_sys_.clear();

#if !TM_HAS_NLOHMANN_JSON
    TM_LOGE("ZoneConfig: nlohmann/json not found. Cannot parse %s", path.c_str());
    TM_LOGE("Hint: add nlohmann/json.hpp to include path (third_party) or install it.");
    return false;
#else
    std::string txt;
    if (!readTextFile_(path, txt)) {
        TM_LOGE("ZoneConfig: failed to open json file: %s", path.c_str());
        return false;
    }

    nlohmann::json j;
    try {
        j = nlohmann::json::parse(txt);
    } catch (const std::exception& e) {
        TM_LOGE("ZoneConfig: json parse error: %s", e.what());
        return false;
    }

    try {
        // 권장 JSON 형태 예시(가정):
        // {
        //   "coordinate_system": "ipm",
        //   "zones": [
        //     { "zone_id": 1, "lane_id": 1, "polygon": [[x,y],[x,y],...] },
        //     ...
        //   ]
        // }
        if (!j.contains("coordinate_system") || !j["coordinate_system"].is_string()) {
            TM_LOGE("ZoneConfig: missing 'coordinate_system' (string).");
            return false;
        }
        coord_sys_ = j["coordinate_system"].get<std::string>();

        if (!j.contains("zones") || !j["zones"].is_array()) {
            TM_LOGE("ZoneConfig: missing 'zones' (array).");
            return false;
        }

        for (const auto& z : j["zones"]) {
            ZonePoly zp;

            if (!z.contains("zone_id") || !z["zone_id"].is_number_integer()) {
                TM_LOGE("ZoneConfig: zone missing 'zone_id' (int).");
                return false;
            }
            zp.zone_id = z["zone_id"].get<int>();

            // lane_id는 "있다고 가정"이지만, 혹시 없을 수도 있으니 기본 -1 처리
            if (z.contains("lane_id") && z["lane_id"].is_number_integer())
                zp.lane_id = z["lane_id"].get<int>();
            else
                zp.lane_id = -1;

            if (!z.contains("polygon") || !z["polygon"].is_array()) {
                TM_LOGE("ZoneConfig: zone_id=%d missing 'polygon' (array).", zp.zone_id);
                return false;
            }

            const auto& arr = z["polygon"];
            if (arr.size() < 3) {
                TM_LOGE("ZoneConfig: zone_id=%d polygon must have >=3 points.", zp.zone_id);
                return false;
            }

            zp.poly.reserve(arr.size());
            for (const auto& p : arr) {
                if (!p.is_array() || p.size() != 2 || !p[0].is_number() || !p[1].is_number()) {
                    TM_LOGE("ZoneConfig: zone_id=%d polygon point must be [x,y].", zp.zone_id);
                    return false;
                }
                float x = p[0].get<float>();
                float y = p[1].get<float>();
                zp.poly.emplace_back(x, y);
            }

            zones_.push_back(std::move(zp));
        }

    } catch (const std::exception& e) {
        TM_LOGE("ZoneConfig: json field error: %s", e.what());
        return false;
    }

    if (!validate_()) {
        TM_LOGE("ZoneConfig: validation failed for %s", path.c_str());
        zones_.clear();
        coord_sys_.clear();
        return false;
    }

    TM_LOGI("ZoneConfig loaded: coord_sys=%s, zones=%zu",
            coord_sys_.c_str(), zones_.size());
    return true;
#endif
}

bool ZoneConfig::validate_() const {
    if (coord_sys_.empty()) {
        TM_LOGE("ZoneConfig validate: coordinate_system empty.");
        return false;
    }

    // 좌표계 명시가 반드시 있어야 하므로, 여기서는 "ipm"만 허용(필요 시 확장)
    if (coord_sys_ != "ipm") {
        TM_LOGW("ZoneConfig validate: coordinate_system='%s' (expected 'ipm').", coord_sys_.c_str());
        // 정책 선택:
        // - strict: return false;
        // - permissive: allow but warn
        // 여기서는 strict하게 막고 싶으면 아래 줄을 주석 해제:
        // return false;
    }

    std::unordered_set<int> seen;
    for (const auto& z : zones_) {
        if (z.zone_id < 0) {
            TM_LOGE("ZoneConfig validate: zone_id must be >=0.");
            return false;
        }
        if (seen.count(z.zone_id)) {
            TM_LOGE("ZoneConfig validate: duplicate zone_id=%d.", z.zone_id);
            return false;
        }
        seen.insert(z.zone_id);

        if ((int)z.poly.size() < 3) {
            TM_LOGE("ZoneConfig validate: zone_id=%d polygon <3 points.", z.zone_id);
            return false;
        }
    }
    return true;
}

} // namespace traffic
