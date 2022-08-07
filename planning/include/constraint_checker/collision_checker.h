#pragma once

#include <vector>
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/obstacle.h"
#include "common/vehicle_param.h"
#include "config/planning_config.h"
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

using StaticObstaclePolygon = math::Polygon2d;
using DynamicObstaclePolygons = std::vector<std::pair<double, math::Polygon2d>>;

class CollisionChecker {
  public:
    CollisionChecker() = default;

    CollisionChecker(const std::vector<const Obstacle *> &obstacles,
                     const ReferenceLine *reference_line,
                     const PlanningConfig *config);

    bool InCollision(double time, const math::Pose &pose,
                     double collision_buffer = 0.0);

  private:
    bool CheckStaticCollision(const math::Box2d &rect);

    bool CheckDynamicCollision(double time, const math::Box2d &rect);

  private:
    // std::vector<const Obstacle *> static_obstacles_;
    // std::vector<const Obstacle *> dynamic_obstacles_;

    const PlanningConfig *config_;

    std::vector<StaticObstaclePolygon> static_obstacles_;
    std::vector<DynamicObstaclePolygons> dynamic_obstacles_;

    std::vector<math::Vec2d> road_barrier_;
};

} // namespace planning
} // namespace mujianhua
