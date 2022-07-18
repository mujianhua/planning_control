#include "tools/vehicle_state.h"

namespace mujianhua {
namespace planning {

VehicleState::VehicleState(const TrajectoryPoint &start_point,
                           const TrajectoryPoint &end_point)
    : start_point_(start_point), end_point_(end_point) {}

const TrajectoryPoint &VehicleState::getStartPoint() const {
    return start_point_;
}

} // namespace planning
} // namespace mujianhua