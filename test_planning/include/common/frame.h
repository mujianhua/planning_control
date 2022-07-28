/**
 * @file
 * @brief
 */

#pragma once

#include "common/vehicle_state.h"
#include "tools/map.h"

namespace mujianhua {
namespace planning {

class Frame {
  public:
    Frame() = default;

    Frame(const VehicleState &vehicle_state, const VehicleState &target_state,
          const Map &map);

    const VehicleState *GetVehicleStartState() const;

    const VehicleState *GetVehicleTargetState() const;

    void UpdateVehicleStartState(VehicleState state);

    void UpdateVehicleTargetState(VehicleState state);

    const Map *GridMap() const;

  private:
    VehicleState vehicle_state_, target_state_;
    Map grid_map_;
};

} // namespace planning
} // namespace mujianhua