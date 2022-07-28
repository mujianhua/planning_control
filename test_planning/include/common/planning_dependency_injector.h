#pragma once

#include "common/vehicle_state.h"
#include "tools/map.h"

namespace mujianhua {
namespace planning {

class PlanningDependencyInjector {
  public:
    PlanningDependencyInjector() = default;
    ~PlanningDependencyInjector() = default;

    VehicleState *vehicle_start_state() { return &start_state_; }

    Map *grid_map() { return &grid_map_; }

  private:
    VehicleState start_state_;
    Map grid_map_;
};

} // namespace planning
} // namespace mujianhua