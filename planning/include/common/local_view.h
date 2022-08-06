#pragma once

#include <memory>
#include <vector>
#include "common/obstacle.h"
#include "planner/planner.h"

namespace mujianhua {
namespace planning {

struct LocalView {
    std::shared_ptr<common::State> vehicle_state;
    std::shared_ptr<common::IndexedObstacles> obstacles;
};

} // namespace planning
} // namespace mujianhua