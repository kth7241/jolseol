// traffic_monitor/src/geometry/polygon.cpp
#include "geometry/polygon.hpp"
#include <cmath>

namespace traffic::geom {

static inline float cross2(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
    // (b-a) x (c-a)
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

static inline bool onSegmentEps(const cv::Point2f& a,
                               const cv::Point2f& b,
                               const cv::Point2f& p,
                               float eps = 1e-4f)
{
    // colinear check
    if (std::fabs(cross2(a, b, p)) > eps) return false;

    // within bounding box
    const float minx = std::min(a.x, b.x) - eps;
    const float maxx = std::max(a.x, b.x) + eps;
    const float miny = std::min(a.y, b.y) - eps;
    const float maxy = std::max(a.y, b.y) + eps;
    return (p.x >= minx && p.x <= maxx && p.y >= miny && p.y <= maxy);
}

bool pointInPolygonRayCasting(const std::vector<cv::Point2f>& poly,
                              const cv::Point2f& p,
                              bool include_boundary)
{
    const int n = (int)poly.size();
    if (n < 3) return false;

    // 경계 포함 옵션이면, 먼저 edge 위 여부 검사
    if (include_boundary) {
        for (int i = 0; i < n; ++i) {
            const cv::Point2f& a = poly[i];
            const cv::Point2f& b = poly[(i + 1) % n];
            if (onSegmentEps(a, b, p)) return true;
        }
    }

    // Ray casting: 오른쪽 방향으로 반직선 쏘고 교차 횟수 홀짝
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const cv::Point2f& pi = poly[i];
        const cv::Point2f& pj = poly[j];

        const bool intersect =
            ((pi.y > p.y) != (pj.y > p.y)) &&
            (p.x < (pj.x - pi.x) * (p.y - pi.y) / ((pj.y - pi.y) + 1e-12f) + pi.x);

        if (intersect) inside = !inside;
    }
    return inside;
}

} // namespace traffic::geom
