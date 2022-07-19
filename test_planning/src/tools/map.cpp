#include "tools/map.h"

namespace mujianhua {
namespace planning {

Map::Map(const grid_map::GridMap &grid_map) : map_(grid_map) {
    if (!grid_map.exists("distance")) {
        ROS_ERROR("grid map must contain distance layer.");
    }
}

double Map::GetObstacleDistance(const Eigen::Vector2d &pos) const {
    if (map_.isInside(pos)) {
        return this->map_.atPosition(
            "distance", pos, grid_map::InterpolationMethods::INTER_LINEAR);
    } else {
        return 0.0;
    }
}

bool Map::IsInside(const Eigen::Vector2d &pos) const {
    return map_.isInside(pos);
}

} // namespace planning
} // namespace mujianhua