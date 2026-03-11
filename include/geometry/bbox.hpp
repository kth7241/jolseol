// traffic_monitor/include/geometry/bbox.hpp
#pragma once

#include <type_traits>
#include <algorithm>   // std::min/max
#include "common/types.hpp"

namespace traffic::geom {

/**
 * 이 파일은 "bbox(바운딩박스) 관련 순수 기하 유틸"만 제공한다.
 * - 알고리즘(검출/추적/속도) 로직 금지
 * - 좌표계: img pixel 좌표계를 기본으로 가정
 *
 * types.hpp 쪽 BBox 구현이 프로젝트마다 다를 수 있으므로,
 * - (x,y,w,h) 스타일 또는 (x,y,width,height) 스타일 모두 대응
 * - cx() 메서드가 있으면 우선 사용, 없으면 x + w/2 로 계산
 */

// ------------------------------
// SFINAE: BBox 필드 접근 유틸
// ------------------------------
namespace detail {

// x
template <typename B>
auto get_x(const B& b) -> decltype(b.x) { return b.x; }

// y
template <typename B>
auto get_y(const B& b) -> decltype(b.y) { return b.y; }

// w (or width)
template <typename B>
auto get_w(const B& b) -> decltype(b.w) { return b.w; }
template <typename B>
auto get_w(const B& b) -> decltype(b.width) { return b.width; }

// h (or height)
template <typename B>
auto get_h(const B& b) -> decltype(b.h) { return b.h; }
template <typename B>
auto get_h(const B& b) -> decltype(b.height) { return b.height; }

// cx() method if exists
template <typename B>
auto get_cx(const B& b) -> decltype(b.cx()) { return b.cx(); }

// fallback cx = x + w/2
template <typename B>
auto get_cx_fallback(const B& b) -> float {
    return static_cast<float>(get_x(b) + 0.5f * get_w(b));
}

// setter: x/y/w/h (or width/height)
template <typename B>
auto set_x(B& b, float v) -> decltype(b.x = v, void()) { b.x = v; }
template <typename B>
auto set_y(B& b, float v) -> decltype(b.y = v, void()) { b.y = v; }

template <typename B>
auto set_w(B& b, float v) -> decltype(b.w = v, void()) { b.w = v; }
template <typename B>
auto set_w(B& b, float v) -> decltype(b.width = v, void()) { b.width = v; }

template <typename B>
auto set_h(B& b, float v) -> decltype(b.h = v, void()) { b.h = v; }
template <typename B>
auto set_h(B& b, float v) -> decltype(b.height = v, void()) { b.height = v; }

// cx() 존재 여부 판정
template <typename B, typename = void>
struct has_cx_method : std::false_type {};
template <typename B>
struct has_cx_method<B, std::void_t<decltype(std::declval<const B&>().cx())>> : std::true_type {};

} // namespace detail

/**
 * bbox_footpoint_img
 * - 입력 bbox(이미지 좌표계)에서 "바닥 중심점(bottom-center)"을 계산한다.
 *
 * 정의:
 *   foot_x = cx = (x + w/2)
 *   foot_y = y + h
 *
 * 좌표계/단위:
 * - 입력: img pixel 좌표 (x,y,w,h 모두 pixel)
 * - 출력: img pixel 좌표 (Pt2f)
 */
traffic::Pt2f bbox_footpoint_img(const traffic::BBox& b);

/**
 * bbox_clip (optional)
 * - bbox를 이미지 경계 안으로 클리핑한다.
 * - bbox가 완전히 밖인 경우 width/height가 0이 될 수 있다.
 *
 * 좌표계/단위:
 * - img_w, img_h: 이미지 크기 (pixel)
 * - bbox: img pixel 좌표계
 *
 * 반환:
 * - 동일 타입(BBox)으로 클리핑된 bbox 반환
 *
 * 주의:
 * - “w/h” 또는 “width/height” 타입 모두 대응
 */
traffic::BBox bbox_clip(const traffic::BBox& b, int img_w, int img_h);

} // namespace traffic::geom
