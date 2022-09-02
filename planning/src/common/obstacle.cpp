
#include "obstacle.h"

#include <algorithm>

#include "math/linear_interpolation.h"

namespace planning {

Obstacle::Obstacle(const std::string &id, const math::Polygon2d &appearance)
    : id_(id), is_static_(true), appearance_(appearance) {}

Obstacle::Obstacle(const std::string &id, const math::Polygon2d &appearance,
                   const obstacle_trajectory &traj)
    : id_(id), is_static_(false), appearance_(appearance), trajectory_(traj) {
  for (const auto &p : traj) {
    std::vector<math::Vec2d> points;
    for (const auto &pt : appearance.points()) {
      points.push_back(p.second.transform({pt.x(), pt.y(), 0.0}));
    }
    math::Polygon2d polygon(points);
    dynamic_obs_.emplace_back(p.first, polygon);
  }
}

// TODO test...
math::Pose Obstacle::GetPoseAtTime(const double relative_time) const {
  auto comp = [](const std::pair<double, math::Pose> p, const double time) {
    return p.first < time;
  };
  auto it_lower = std::lower_bound(trajectory_.begin(), trajectory_.end(),
                                   relative_time, comp);
  if (it_lower == trajectory_.begin()) {
    return trajectory_.begin()->second;
  } else if (it_lower == trajectory_.end()) {
    return trajectory_.rbegin()->second;
  }
  return math::InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower,
                                                   relative_time);
}

math::Polygon2d Obstacle::GetPolygonAtTime(double relative_time) const {
  const math::Pose pose = GetPoseAtTime(relative_time);
  std::vector<math::Vec2d> points;
  for (const auto &pt : appearance_.points()) {
    points.push_back(pose.transform({pt.x(), pt.y(), 0.0}));
  }
  math::Polygon2d polygon(points);
  return polygon;
}

}  // namespace planning
