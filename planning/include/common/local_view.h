#pragma once

#include <memory>
#include <vector>
#include "common/obstacle.h"
#include "common/vehicle_state.h"

namespace mujianhua {
namespace planning {

struct LocalView {
    std::shared_ptr<State> vehicle_state;
    std::shared_ptr<IndexedObstacles> obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    std::vector<math::Polygon2d> static_obstacles;
};

} // namespace planning
} // namespace mujianhua