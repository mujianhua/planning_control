#pragma once

#include "planner/planner.h"

namespace mujianhua {
namespace planning {

struct LocalView {
    std::shared_ptr<State> vehicle_state;
};

} // namespace planning
} // namespace mujianhua