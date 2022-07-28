#include "common/frame.h"
#include "common/vehicle_state2.h"

namespace mujianhua {
namespace planning {

Frame::Frame(const VehicleState2 &vehicle_state,
             const VehicleState2 &target_state, const Map &map)
    : vehicle_state_(vehicle_state), target_state_(target_state),
      grid_map_(map) {}

const VehicleState2 *Frame::GetVehicleStartState() const {
    return &vehicle_state_;
}

const VehicleState2 *Frame::GetVehicleTargetState() const {
    return &target_state_;
}

void Frame::UpdateVehicleStartState(VehicleState2 state) {
    vehicle_state_ = state;
}

void Frame::UpdateVehicleTargetState(VehicleState2 state) {
    target_state_ = state;
}

const Map *Frame::GridMap() const { return &grid_map_; }

} // namespace planning
} // namespace mujianhua