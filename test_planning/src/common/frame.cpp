#include "common/frame.h"
#include "common/vehicle_state2.h"

namespace mujianhua {
namespace planning {

Frame::Frame(const VehicleState2 &vehicle_state, const Map &map)
    : vehicle_state_(vehicle_state), grid_map_(map) {}

const VehicleState2 *Frame::GetVehicleStartState() const {
    return &vehicle_state_;
}

const Map *Frame::GridMap() const { return &grid_map_; }

} // namespace planning
} // namespace mujianhua