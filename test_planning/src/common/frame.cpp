#include "common/frame.h"
#include "common/vehicle_state.h"

namespace mujianhua {
namespace planning {

Frame::Frame(const VehicleState &vehicle_state,
             const VehicleState &target_state, const Map &map)
    : vehicle_state_(vehicle_state), target_state_(target_state),
      grid_map_(map) {}

const VehicleState *Frame::GetVehicleStartState() const {
    return &vehicle_state_;
}

const VehicleState *Frame::GetVehicleTargetState() const {
    return &target_state_;
}

void Frame::UpdateVehicleStartState(VehicleState state) {
    vehicle_state_ = state;
}

void Frame::UpdateVehicleTargetState(VehicleState state) {
    target_state_ = state;
}

const Map *Frame::GridMap() const { return &grid_map_; }

} // namespace planning
} // namespace mujianhua