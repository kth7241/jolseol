// traffic_monitor/src/geometry/bbox.cpp
#include "geometry/bbox.hpp"

namespace traffic::geom {

traffic::Pt2f bbox_footpoint_img(const traffic::BBox& b) {
    // cx(): 있으면 우선 사용, 없으면 x + w/2 계산
    float cx = 0.f;
    if constexpr (detail::has_cx_method<traffic::BBox>::value) {
        cx = static_cast<float>(detail::get_cx(b));
    } else {
        cx = detail::get_cx_fallback(b);
    }

    const float y  = static_cast<float>(detail::get_y(b));
    const float h  = static_cast<float>(detail::get_h(b));
    const float fy = y + h; // bottom edge (y + h)

    // Pt2f가 어떤 타입이든 (x,y) 생성이 가능하다는 가정(보통 cv::Point2f)
    return traffic::Pt2f{cx, fy};
}

traffic::BBox bbox_clip(const traffic::BBox& b, int img_w, int img_h) {
    // img_w/img_h가 비정상이면 원본 반환(예외 대신 안전 처리)
    if (img_w <= 0 || img_h <= 0) return b;

    // bbox를 float x,y,w,h로 통일해서 계산
    float x = static_cast<float>(detail::get_x(b));
    float y = static_cast<float>(detail::get_y(b));
    float w = static_cast<float>(detail::get_w(b));
    float h = static_cast<float>(detail::get_h(b));

    // 오른쪽/아래쪽 끝 좌표(half-open가 아닌 “끝점” 기준으로 계산)
    float x2 = x + w;
    float y2 = y + h;

    // 클리핑 범위: [0, img_w], [0, img_h]
    // (bbox 표현이 [x, x+w] 형태라고 가정)
    x  = std::max(0.f, std::min(x,  static_cast<float>(img_w)));
    y  = std::max(0.f, std::min(y,  static_cast<float>(img_h)));
    x2 = std::max(0.f, std::min(x2, static_cast<float>(img_w)));
    y2 = std::max(0.f, std::min(y2, static_cast<float>(img_h)));

    w = std::max(0.f, x2 - x);
    h = std::max(0.f, y2 - y);

    traffic::BBox out = b;
    detail::set_x(out, x);
    detail::set_y(out, y);
    detail::set_w(out, w);
    detail::set_h(out, h);
    return out;
}

} // namespace traffic::geom
