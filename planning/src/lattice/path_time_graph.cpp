/**
 * \file path_time_graph.cc
 */

#include "path_time_graph.h"

#include <algorithm>

#include "common/planning_config.h"
#include "common/planning_gflags.h"
#include "common/st_point.h"

namespace planning {

PathTimeGraph::PathTimeGraph(const std::vector<const Obstacle *> &obstacles,
                             const ReferenceLine *reference_line,
                             double s_start, double s_end, double t_start,
                             double t_end,
                             const std::array<double, 3> &init_d) {
  path_range_.first = s_start;
  path_range_.second = s_end;
  time_range_.first = t_start;
  time_range_.second = t_end;
  init_d_ = init_d;

  SetupObstacles(obstacles, reference_line);
}

bool PathTimeGraph::GetPathTimeObstacle(const std::string &obstacle_id,
                                        STBoundary *path_time_obstacle) {
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end())
    return false;
  *path_time_obstacle = path_time_obstacle_map_[obstacle_id];
  return true;
}

std::pair<double, double> PathTimeGraph::get_path_range() const {
  return path_range_;
}

std::pair<double, double> PathTimeGraph::get_time_range() const {
  return time_range_;
}

bool PathTimeGraph::IsObstacleInGraph(const std::string &obstacle_id) {
  return path_time_obstacle_map_.find(obstacle_id) !=
         path_time_obstacle_map_.end();
}

void PathTimeGraph::SetupObstacles(
    const std::vector<const Obstacle *> &obstacles,
    const ReferenceLine *reference_line) {
  for (const Obstacle *obstacle : obstacles) {
    if (obstacle->IsStatic()) {
      SetStaticObstacle(obstacle, reference_line);
    } else {
      SetDynamicObstacle(obstacle, reference_line);
    }
  }
}

std::vector<std::pair<double, double>> PathTimeGraph::GetLateralBounds(
    double s_start, double s_end, double s_resolution) {
  double s_range = s_end - s_start;
  double s_curr = s_start;
  auto num_bound = static_cast<size_t>(s_range / s_resolution);
  std::vector<std::pair<double, double>> bounds;
  std::vector<double> discretized_path;

  // TODO: new a class...
  double ego_width = VehicleParam().width;

  double ego_d_lower = init_d_[0] - ego_width / 2.0;
  double ego_d_upper = init_d_[0] + ego_width / 2.0;
  // initialize bounds by reference line.
  for (size_t i = 0; i != num_bound; ++i) {
    // TODO: left and right width according reference line.
    double left_width = 1.75;
    double right_width = 1.75;

    bounds.emplace_back(
        std::min(-right_width, ego_d_lower - FLAGS_bound_buffer),
        std::max(left_width, ego_d_upper + FLAGS_bound_buffer));
    discretized_path.push_back(s_curr);
    s_curr += s_resolution;
  }

  for (const SLBoundary &static_sl_boundary : static_obs_sl_boundaries_) {
    UpdateLateralBoundsByObstacle(static_sl_boundary, discretized_path, s_start,
                                  s_end, &bounds);
  }

  // calculate center of vehicle bounds.
  for (size_t i = 0; i != bounds.size(); ++i) {
    bounds[i].first += ego_width / 2.0;
    bounds[i].second -= ego_width / 2.0;
    if (bounds[i].first >= bounds[i].second) {
      bounds[i].first = 0.0;
      bounds[i].second = 0.0;
    }
  }

  return bounds;
}

SLBoundary PathTimeGraph::ComputeObstacleBoundary(
    const std::vector<math::Vec2d> &vertices,
    const ReferenceLine *reference_line) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  for (const auto &point : vertices) {
    auto sl_point = reference_line->XYToSL(point);
    start_s = std::fmin(start_s, sl_point.first);
    end_s = std::fmax(end_s, sl_point.first);
    start_l = std::fmin(start_l, sl_point.second);
    end_l = std::fmax(end_l, sl_point.second);
  }
  return SLBoundary(start_s, end_s, start_l, end_l);
}

void PathTimeGraph::SetStaticObstacle(const Obstacle *obstacle,
                                      const ReferenceLine *reference_line) {
  const math::Polygon2d &polygon = obstacle->polygon2d();
  const std::string &obstacle_id = obstacle->Id();

  SLBoundary sl_boundary =
      ComputeObstacleBoundary(polygon.GetAllVertices(), reference_line);

  path_time_obstacle_map_[obstacle_id].set_id(obstacle_id);
  path_time_obstacle_map_[obstacle_id].set_bottom_left_point(
      SetPathTimePoint(obstacle_id, sl_boundary.start_s, 0.0));
  path_time_obstacle_map_[obstacle_id].set_bottom_right_point(SetPathTimePoint(
      obstacle_id, sl_boundary.start_s, FLAGS_trajectory_time_length));
  path_time_obstacle_map_[obstacle_id].set_upper_left_point(
      SetPathTimePoint(obstacle_id, sl_boundary.end_s, 0.0));
  path_time_obstacle_map_[obstacle_id].set_upper_right_point(SetPathTimePoint(
      obstacle_id, sl_boundary.end_s, FLAGS_trajectory_time_length));
  static_obs_sl_boundaries_.push_back(std::move(sl_boundary));
}

void PathTimeGraph::SetDynamicObstacle(const Obstacle *obstacle,
                                       const ReferenceLine *reference_line) {
  const std::string &obstacle_id = obstacle->Id();
  double relative_time = time_range_.first;
  while (relative_time < time_range_.second) {
    math::Polygon2d polygon = obstacle->GetPolygonAtTime(relative_time);
    SLBoundary sl_boundary =
        ComputeObstacleBoundary(polygon.points(), reference_line);

    if (sl_boundary.start_s > path_range_.second ||
        sl_boundary.end_s < path_range_.first) {
      // path_time_obstacle_map have this obstacle, and it's over path range.
      if (path_time_obstacle_map_.find(obstacle_id) !=
          path_time_obstacle_map_.end())
        break;
      relative_time += FLAGS_trajectory_time_resolution;
      continue;
    }
    // path_time_obstacle_map don't have this obstacle.
    if (path_time_obstacle_map_.find(obstacle_id) ==
        path_time_obstacle_map_.end()) {
      path_time_obstacle_map_[obstacle_id].set_id(obstacle_id);
      path_time_obstacle_map_[obstacle_id].set_bottom_left_point(
          SetPathTimePoint(obstacle_id, sl_boundary.start_s, relative_time));
      path_time_obstacle_map_[obstacle_id].set_upper_left_point(
          SetPathTimePoint(obstacle_id, sl_boundary.end_s, relative_time));
    }
    // path_time_obstacle_map obtain this obstacle.
    path_time_obstacle_map_[obstacle_id].set_bottom_right_point(
        SetPathTimePoint(obstacle_id, sl_boundary.start_s, relative_time));
    path_time_obstacle_map_[obstacle_id].set_upper_right_point(
        SetPathTimePoint(obstacle_id, sl_boundary.end_s, relative_time));

    relative_time += FLAGS_trajectory_time_resolution;
  }
}

STPoint PathTimeGraph::SetPathTimePoint(const std::string &obstacle_id,
                                        const double s, const double t) const {
  STPoint path_time_point(s, t);
  return path_time_point;
}

void PathTimeGraph::UpdateLateralBoundsByObstacle(
    const SLBoundary &sl_boundary, const std::vector<double> &discretized_path,
    const double s_start, const double s_end,
    std::vector<std::pair<double, double>> *const bounds) {
  if (sl_boundary.start_s > s_end || sl_boundary.end_s < s_start) return;
  auto start_iter = std::lower_bound(
      discretized_path.begin(), discretized_path.end(), sl_boundary.start_s);
  auto end_iter = std::upper_bound(
      discretized_path.begin(), discretized_path.end(),
      sl_boundary.start_s);  // TODO: I think this is apollo coding error.
  size_t start_index = start_iter - discretized_path.begin();
  size_t end_index = end_iter - discretized_path.end();

  // it's a obstacle on reference line.
  if (sl_boundary.end_l > -FLAGS_numerical_epsilon &&
      sl_boundary.start_l < FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i != end_index; ++i) {
      bounds->at(i).first = -FLAGS_numerical_epsilon;
      bounds->at(i).second = FLAGS_numerical_epsilon;
    }
    return;
  }

  // it's a obstacle locate left of reference line.
  if (sl_boundary.end_l < FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size());
         ++i) {
      bounds->at(i).first =
          std::max(bounds->at(i).first, sl_boundary.end_l + FLAGS_nudge_buffer);
    }
    return;
  }

  // it's a obstacle locate left of reference line.
  if (sl_boundary.start_l > -FLAGS_numerical_epsilon) {
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size());
         ++i) {
      bounds->at(i).second = std::min(bounds->at(i).second,
                                      sl_boundary.start_l - FLAGS_nudge_buffer);
    }
    return;
  }
}

}  // namespace planning
