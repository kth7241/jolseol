// traffic_monitor/include/fusion/footpoint_mapper.hpp
#pragma once

#include <vector>
#include <type_traits>
#include "common/types.hpp"
#include "ipm/ipm_warper.hpp"   // 허용됨 (요구사항)

namespace traffic {

/**
 * FootpointMapper
 * - 책임: Track(bbox_img) → TrackFoot(foot_img, foot_ipm)로 "좌표 매핑"만 수행
 * - 속도 계산 / 필터 / 트래커 안정화 로직 절대 금지
 *
 * 좌표계:
 * - bbox_img: 이미지(img) 픽셀 좌표계
 * - foot_img: 이미지(img) 픽셀 좌표계 (bottom-center)
 * - foot_ipm: IPM(top-view) 픽셀 좌표계 (warpPoint 결과)
 *
 * 단위:
 * - 모든 좌표는 pixel 단위
 * - meters 변환은 SpeedEstimator에서 meters_per_pixel로 수행(여기서는 하지 않음)
 */
class FootpointMapper {
public:
    FootpointMapper() = default;

    /**
     * map
     * @param tracks    tracker 결과(각 Track는 bbox + track_id + timestamp 포함 가정)
     * @param warper    IPM 변환기 (warpPoint 제공)
     * @return TrackFoot 리스트 (track_id별 foot_img, foot_ipm 채움)
     */
    std::vector<TrackFoot> map(const std::vector<Track>& tracks,
                               const IPMWarper& warper) const;

    /**
     * (optional) zone/lane을 TrackFoot에 채우는 오버로드
     * - include/zone/zone_manager.hpp 의존은 "이 함수 선언부에서만" 분리하기 위해
     *   아래에 별도 include를 둔다.
     *
     * 주의:
     * - TrackFoot 타입에 zone_id/lane_id 필드가 "실제로 있을 때만" 세팅된다.
     *   (없으면 컴파일되되 아무 것도 하지 않음: SFINAE)
     */
    std::vector<TrackFoot> map(const std::vector<Track>& tracks,
                               const IPMWarper& warper,
                               const class ZoneManager& zone_mgr) const;
};

} // namespace traffic
