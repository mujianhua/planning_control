#include "tools/vehicle_state.h"

namespace mujianhua {
namespace planning {

VehicleState::VehicleState(const TrajectoryPoint &start_point,
                           const TrajectoryPoint &end_point)
    : start_point_(start_point), end_point_(end_point) {}

void VehicleState::SetStartPoint(const TrajectoryPoint &point) {
    start_point_ = point;
}

void VehicleState::SetTargetPoint(const TrajectoryPoint &point) {
    end_point_ = point;
}

const TrajectoryPoint &VehicleState::getStartPoint() const {
    return start_point_;
}

const TrajectoryPoint &VehicleState::getTargetPoint() const {
    return end_point_;
}

void VehicleState::SetInitError(double init_offset, double init_heading_error) {
    init_offset_ = init_offset;
    init_heading_error_ = init_heading_error;
}

std::vector<double> VehicleState::GetInitError() const {
    return {init_offset_, init_heading_error_};
}

} // namespace planning
} // namespace mujianhua