
#include "reference_line.h"

#include <algorithm>

#include "../common/data_struct.h"
#include "../math/math_utils.h"

namespace planning {

ReferenceLine::ReferenceLine(const ReferenceLine &rhs, size_t begin,
                             size_t end) {
  if (end < 0) {
    end = rhs.reference_points_.size();
  }
  reference_points_.resize(end - begin);
  std::copy_n(std::next(rhs.reference_points_.begin(), begin),
              reference_points_.size(), reference_points_.begin());
}

std::vector<TrajectoryPoint>::const_iterator
ReferenceLine::QueryLowerBoundStationPoint(double station) const {
  if (station >= reference_points_.back().s) {
    return reference_points_.end() - 1;
  } else if (station < reference_points_.front().s) {
    return reference_points_.begin();
  }

  return std::lower_bound(
      reference_points_.begin(), reference_points_.end(), station,
      [](const TrajectoryPoint &t, double station) { return t.s < station; });
}

TrajectoryPoint LinearInterpolateTrajectory(const TrajectoryPoint &p0,
                                            const TrajectoryPoint &p1,
                                            double s) {
  double s0 = p0.s;
  double s1 = p1.s;
  if (std::abs(s1 - s0) < math::kMathEpsilon) {
    return p0;
  }

  TrajectoryPoint pt;
  double weight = (s - s0) / (s1 - s0);
  pt.s = s;
  pt.x = (1 - weight) * p0.x + weight * p1.x;
  pt.y = (1 - weight) * p0.y + weight * p1.y;
  pt.theta = math::slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  pt.kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  pt.velocity = (1 - weight) * p0.velocity + weight * p1.velocity;
  pt.left_bound = (1 - weight) * p0.left_bound + weight * p1.left_bound;
  pt.right_bound = (1 - weight) * p0.right_bound + weight * p1.right_bound;

  return pt;
}

TrajectoryPoint ReferenceLine::GetMatchPoint(double station) const {
  auto iter = QueryLowerBoundStationPoint(station);

  if (iter == reference_points_.begin()) {
    iter = std::next(iter);
  }

  auto prev = std::prev(iter, 1);

  return LinearInterpolateTrajectory(*prev, *iter, station);
}

std::vector<TrajectoryPoint>::const_iterator ReferenceLine::QueryNearestPoint(
    const Vec2d &point, double *out_distance) const {
  auto nearest_iter = reference_points_.begin();
  double nearest_distance = std::numeric_limits<double>::max();

  for (auto iter = reference_points_.begin(); iter != reference_points_.end();
       iter++) {
    double dx = iter->x - point.x(), dy = iter->y - point.y();
    double distance = dx * dx + dy * dy;
    if (distance < nearest_distance) {
      nearest_iter = iter;
      nearest_distance = distance;
    }
  }

  if (out_distance != nullptr) {
    *out_distance = sqrt(nearest_distance);
  }
  return nearest_iter;
}

TrajectoryPoint ReferenceLine::GetProjection(const Vec2d &xy) const {
  long point_idx =
      std::distance(reference_points_.begin(), QueryNearestPoint(xy));
  auto project_point = reference_points_[point_idx];
  auto index_start = std::max(0l, point_idx - 1);
  auto index_end = std::min(reference_points_.size() - 1, (ulong)point_idx + 1);

  if (index_start < index_end) {
    double v0x = xy.x() - reference_points_[index_start].x;
    double v0y = xy.y() - reference_points_[index_start].y;

    double v1x =
        reference_points_[index_end].x - reference_points_[index_start].x;
    double v1y =
        reference_points_[index_end].y - reference_points_[index_start].y;

    double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
    double dot = v0x * v1x + v0y * v1y;

    double delta_s = dot / v1_norm;
    project_point = LinearInterpolateTrajectory(
        reference_points_[index_start], reference_points_[index_end],
        reference_points_[index_start].s + delta_s);
  }
  return project_point;
}

std::pair<double, double> ReferenceLine::XYToSL(const Vec2d &xy) const {
  auto project_point = GetProjection(xy);
  double nr_x = xy.x() - project_point.x, nr_y = xy.y() - project_point.y;
  double lateral =
      copysign(hypot(nr_x, nr_y), nr_y * cos(project_point.theta) -
                                      nr_x * sin(project_point.theta));
  return {project_point.s, lateral};
}

Vec2d ReferenceLine::GetCartesian(double station, double lateral) const {
  auto ref = GetMatchPoint(station);
  return {ref.x - lateral * sin(ref.theta), ref.y + lateral * cos(ref.theta)};
}

}  // namespace planning
