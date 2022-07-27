/**
 * @file
 * @brief
 */

#pragma once

#include "common/vehicle_state2.h"
#include "tools/map.h"

namespace mujianhua {
namespace planning {

class Frame {
  public:
    Frame() = default;

    Frame(const VehicleState2 &vehicle_state, const Map &map);

    const VehicleState2 *GetVehicleStartState() const;

    const Map *GridMap() const;

  private:
    VehicleState2 vehicle_state_;
    Map grid_map_;
};

} // namespace planning
} // namespace mujianhua