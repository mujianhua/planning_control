
#pragma once

#include <array>
#include <unordered_map>
#include <vector>

#include "../common/data_struct.h"
#include "../common/st_boundary.h"
#include "../common/st_point.h"
#include "../planning/obstacle.h"
#include "../reference_line/reference_line.h"

namespace planning {

class PathTimeGraph {
 public:
  PathTimeGraph(const std::vector<const Obstacle *> &obstacles,
                const ReferenceLine *reference_line, double s_start,
                double s_end, double t_start, double t_end,
                const std::array<double, 3> &init_d);

 private:
  void SetupObstacles(const std::vector<const Obstacle *> &obstacles,
                      const ReferenceLine *reference_line);

  SLBoundary ComputeObstacleBoundary(const std::vector<math::Vec2d> &vertices,
                                     const ReferenceLine *reference_line) const;

  void SetStaticObstacle(const Obstacle *obstacle,
                         const ReferenceLine *reference_line);

  void SetDynamicObstacle(const Obstacle *obstacle,
                          const ReferenceLine *reference_line);

 private:
  std::pair<double, double> path_range_;
  std::pair<double, double> time_range_;
  std::array<double, 3> init_d_{};
};

}  // namespace planning
