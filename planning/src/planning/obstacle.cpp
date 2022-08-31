
#include "obstacle.h"

namespace planning {

Obstacle::Obstacle(const std::string &id, const StaticObstacle &obs)
    : id_(id), is_static_(true), static_obs_(obs) {}

Obstacle::Obstacle(const std::string &id, const DynamicObstacle &obs)
    : id_(id), is_static_(false), dynamic_obs_(obs) {}

Obstacle::Obstacle(const std::string &id, const math::Polygon2d &appearance,
                   const obstacle_trajectory &traj)
    : id_(id), is_static_(false), appearance_(appearance) {}

math::Polygon2d Obstacle::GetPolygon2dAtTime(
    const double relvative_time) const {}

}  // namespace planning
