// traffic_monitor/src/fusion/footpoint_mapper.cpp
#include "fusion/footpoint_mapper.hpp"
#include "geometry/bbox.hpp"

// optional overload 구현을 위해 .cpp에서만 include (규약 준수)
#include "zone/zone_manager.hpp"

#include <type_traits>
#include <utility>

namespace traffic {

// ------------------------------
// Track / TrackFoot 필드 접근 SFINAE
// ------------------------------
namespace detail {

// Track에서 bbox 얻기: t.bbox 또는 t.bbox_img 둘 다 대응
template <typename T>
auto get_bbox(const T& t) -> decltype(t.bbox) { return t.bbox; }
template <typename T>
auto get_bbox(const T& t) -> decltype(t.bbox_img) { return t.bbox_img; }

// Track에서 id 얻기: t.track_id
template <typename T>
auto get_id(const T& t) -> decltype(t.track_id) { return t.track_id; }

// Track에서 class_id 얻기: 없으면 -1
template <typename T>
auto get_class_id(const T& t) -> decltype(t.class_id) { return t.class_id; }
inline int get_class_id(...) { return -1; }

// Track에서 timestamp 얻기: t.ts_ns 또는 t.ts 둘 다 대응
template <typename T>
auto get_ts(const T& t) -> decltype(t.ts_ns) { return t.ts_ns; }
template <typename T>
auto get_ts(const T& t) -> decltype(t.ts) { return t.ts; }

// TrackFoot에 zone_id/lane_id 필드가 있으면 세팅
// - zone/lane을 찾지 못했을 때는 nullopt로 비워 둔다.
template <typename TF>
auto set_zone_id(TF& tf, int z) -> decltype(tf.zone_id = std::optional<int>{}, void()) {
    if (z >= 0) tf.zone_id = z;
    else        tf.zone_id.reset();
}

template <typename TF>
auto set_lane_id(TF& tf, int l) -> decltype(tf.lane_id = std::optional<int>{}, void()) {
    if (l >= 0) tf.lane_id = l;
    else        tf.lane_id.reset();
}

// zone_id/lane_id 필드 존재 여부
template <typename TF, typename = void>
struct has_zone_lane : std::false_type {};

template <typename TF>
struct has_zone_lane<TF, std::void_t<
    decltype(std::declval<TF&>().zone_id),
    decltype(std::declval<TF&>().lane_id)
>> : std::true_type {};

} // namespace detail

std::vector<TrackFoot> FootpointMapper::map(const std::vector<Track>& tracks,
                                            const IPMWarper& warper) const
{
    std::vector<TrackFoot> out;
    out.reserve(tracks.size());

    for (const auto& tr : tracks) {
        TrackFoot tf;

        // 추적 ID / 클래스 / 시각 전달
        tf.track_id = detail::get_id(tr);
        tf.class_id = detail::get_class_id(tr);
        tf.ts_ns = static_cast<TimestampNs>(detail::get_ts(tr));

        // 1) bbox(img px) -> foot_img (bottom-center)
        //    프로젝트 전체에서 바닥점은 "bbox 하단 중앙"으로 통일한다.
        const auto bbox = detail::get_bbox(tr);
        tf.foot_img = traffic::geom::bbox_footpoint_img(bbox);

        // 2) foot_img -> foot_ipm (IPM px)
        tf.foot_ipm = warper.warpPoint(tf.foot_img);

        out.push_back(tf);
    }
    return out;
}

std::vector<TrackFoot> FootpointMapper::map(const std::vector<Track>& tracks,
                                            const IPMWarper& warper,
                                            const ZoneManager& zone_mgr) const
{
    // 먼저 좌표 매핑만 수행
    auto feet = map(tracks, warper);

    // 그 다음, 이미 계산된 IPM 바닥점을 사용해 zone/lane 판정만 덧붙인다.
    // 즉 이 함수도 "좌표 매핑 + zone 태깅"만 담당하고,
    // 속도 추정이나 시간 필터링은 절대 하지 않는다.
    for (auto& tf : feet) {
        if constexpr (detail::has_zone_lane<TrackFoot>::value) {
            const int zid = zone_mgr.zoneOf(tf.foot_ipm);  // IPM px 기준
            const int lid = zone_mgr.laneOf(tf.foot_ipm);  // IPM px 기준
            detail::set_zone_id(tf, zid);
            detail::set_lane_id(tf, lid);
        } else {
            // 혹시 TrackFoot 타입이 바뀌어 zone/lane 필드가 사라져도
            // 전체 빌드가 깨지지 않도록 조용히 통과한다.
            (void)zone_mgr;
            break;
        }
    }
    return feet;
}

} // namespace traffic
