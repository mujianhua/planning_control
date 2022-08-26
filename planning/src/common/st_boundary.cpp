
#include "st_boundary.h"

#include "../math/line_segment2d.h"
#include "../math/polygon2d.h"

namespace planning {

STBoundary::STBoundary(
    const std::vector<std::pair<STPoint, STPoint>> &point_pairs,
    bool is_accurate_boundary) {
  std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);
  if (!is_accurate_boundary) {
    RemoveRedundantPoints(&point_pairs);
  }

  for (const auto &item : reduced_pairs) {
    const double t = item.first.t();
    lower_points_.emplace_back(item.first.s(), t);
    upper_points_.emplace_back(item.second.s(), t);
  }

  // polygon
  for (const auto &point : lower_points_) {
    points_.emplace_back(point.t(), point.s());
  }
  for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); rit++) {
    points_.emplace_back(rit->t(), rit->s());
  }
  BuildFromPoints();

  for (const auto &point : lower_points_) {
    min_s_ = std::fmin(min_s_, point.s());
  }
  for (const auto &point : upper_points_) {
    max_s_ = std::fmax(max_s_, point.s());
  }
  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();
}

void STBoundary::set_bottom_left_point(STPoint st_point) {
  bottom_left_point_ = st_point;
}
void STBoundary::set_bottom_right_point(STPoint st_point) {
  bottom_right_point_ = st_point;
}
void STBoundary::set_upper_left_point(STPoint st_point) {
  upper_left_point_ = st_point;
}
void STBoundary::set_upper_right_point(STPoint st_point) {
  upper_right_point_ = st_point;
}

///////////////////////////////////////////////////////////////////////////////
// Private functions for internal usage.

bool STBoundary::IsPointNear(const math::LineSegment2d &seg,
                             const math::Vec2d &point, const double max_dist) {
  return seg.DistanceSquareTo(point) < max_dist * max_dist;
}

void STBoundary::RemoveRedundantPoints(
    const std::vector<std::pair<STPoint, STPoint>> *point_pairs) {
  if (!point_pairs || point_pairs->size() <= 2) {
    return;
  }
  const double kMaxDist = 0.1;
  size_t i = 0;
  size_t j = 1;
  while (i < point_pairs->size() && j + 1 < point_pairs->size()) {
    math::LineSegment2d lower_seg(point_pairs->at(i).first,
                                  point_pairs->at(j + 1).first);
    math::LineSegment2d upper_seg(point_pairs->at(i).second,
                                  point_pairs->at(j + 1).second);
    if (!IsPointNear(lower_seg, point_pairs->at(j).first, kMaxDist) ||
        !IsPointNear(upper_seg, point_pairs->at(j).second, kMaxDist)) {
      ++i;
    }
    ++j;
  }
}

}  // namespace planning
