
#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/data_struct.h"
#include "common/obstacle.h"
#include "common/st_boundary.h"
#include "common/st_point.h"
#include "reference_line/reference_line.h"

namespace planning {

class PathTimeGraph {
 public:
  PathTimeGraph(const std::vector<const Obstacle *> &obstacles,
                const ReferenceLine *reference_line, double s_start,
                double s_end, double t_start, double t_end,
                const std::array<double, 3> &init_d);

  bool GetPathTimeObstacle(const std::string &obstacle_id,
                           STBoundary *path_time_obstacle);

  std::pair<double, double> get_path_range() const;

  std::pair<double, double> get_time_range() const;

  bool IsObstacleInGraph(const std::string &obstacle_id);

  /**
   * <right, left>, <right, left>...
   */
  std::vector<std::pair<double, double>> GetLateralBounds(double s_start,
                                                          double s_end,
                                                          double s_resolution);

 private:
  void SetupObstacles(const std::vector<const Obstacle *> &obstacles,
                      const ReferenceLine *reference_line);

  SLBoundary ComputeObstacleBoundary(const std::vector<math::Vec2d> &vertices,
                                     const ReferenceLine *reference_line) const;

  STPoint SetPathTimePoint(const std::string &obstacle_id, const double s,
                           const double t) const;

  void SetStaticObstacle(const Obstacle *obstacle,
                         const ReferenceLine *reference_line);

  void SetDynamicObstacle(const Obstacle *obstacle,
                          const ReferenceLine *reference_line);

  void UpdateLateralBoundsByObstacle(
      const SLBoundary &sl_boundary,
      const std::vector<double> &discretized_path, const double s_start,
      const double s_end, std::vector<std::pair<double, double>> *const bounds);

 private:
  std::pair<double, double> path_range_;
  std::pair<double, double> time_range_;
  std::array<double, 3> init_d_{};

  std::unordered_map<std::string, STBoundary> path_time_obstacle_map_;
  std::vector<STBoundary> path_time_obstacles_;
  std::vector<SLBoundary> static_obs_sl_boundaries_;
};

}  // namespace planning
