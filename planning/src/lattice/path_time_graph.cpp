
#include "path_time_graph.h"

#include "../math/path_matcher.h"
#include "../math/polygon2d.h"

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
}

void PathTimeGraph::SetDynamicObstacle(const Obstacle *obstacle,
                                       const ReferenceLine *reference_line) {}

}  // namespace planning
