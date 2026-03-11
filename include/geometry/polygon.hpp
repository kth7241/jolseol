// traffic_monitor/include/geometry/polygon.hpp
#pragma once
#include <opencv2/core.hpp>
#include <vector>

namespace traffic::geom {

/**
 * pointInPolygonRayCasting
 * - Ray casting 방식 point-in-polygon 테스트
 *
 * @param poly  폐곡선 폴리곤(정점 리스트). 최소 3개 필요.
 * @param p     테스트할 점 (IPM 좌표계 픽셀)
 * @param include_boundary 경계선 위의 점을 내부로 볼지 여부
 * @return true면 내부(또는 include_boundary 조건에 따라 경계 포함)
 *
 * 좌표계/단위:
 * - poly와 p 모두 동일 좌표계(IPM pixel)라고 가정
 * 예외/주의:
 * - poly.size() < 3 이면 항상 false
 */
bool pointInPolygonRayCasting(const std::vector<cv::Point2f>& poly,
                              const cv::Point2f& p,
                              bool include_boundary = true);

} // namespace traffic::geom
