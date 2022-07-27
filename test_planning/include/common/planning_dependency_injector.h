#pragma once

#include "common/vehicle_state2.h"
#include "tools/map.h"

namespace mujianhua {
namespace planning {

class PlanningDependencyInjector {
  public:
    PlanningDependencyInjector() = default;
    ~PlanningDependencyInjector() = default;

    VehicleState2 *vehicle_start_state() { return &start_state_; }

    Map *grid_map() { return &grid_map_; }

  private:
    VehicleState2 start_state_;
    Map grid_map_;
};

} // namespace planning
} // namespace mujianhua