#include "tools/vehicle_state.h"

namespace mujianhua {
namespace planning {

VehicleState::VehicleState(const TrajectoryPoint &start_point,
                           const TrajectoryPoint &end_point)
    : start_point_(start_point), end_point_(end_point) {}

const TrajectoryPoint &VehicleState::getStartPoint() const {
    return start_point_;
}

const TrajectoryPoint &VehicleState::getTargetPoint() const {
    return end_point_;
}

void VehicleState::SetInitError(double init_offest, double init_heading_error) {
    init_offest_ = init_offest;
    init_heading_error_ = init_heading_error;
}

} // namespace planning
} // namespace mujianhua