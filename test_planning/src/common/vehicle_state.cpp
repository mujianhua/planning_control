#include "common/vehicle_state.h"

namespace mujianhua {
namespace planning {

VehicleState::VehicleState(double x, double y, double heading)
    : x_(x), y_(y), heading_(heading) {}

void VehicleState::Update(const double &x, const double &y,
                          const double &heading) {
    x_ = x;
    y_ = y;
    heading_ = heading;
}

const double &VehicleState::x() const { return x_; }

const double &VehicleState::y() const { return y_; }

const double &VehicleState::heading() const { return heading_; }

const double &VehicleState::lateral_error() const { return lateral_error_; }

const double &VehicleState::heading_error() const { return heading_error_; }

double &VehicleState::x() { return x_; }

double &VehicleState::y() { return y_; }

double &VehicleState::heading() { return heading_; }

void VehicleState::SetInitialError(double lateral_error, double heading_error) {
    lateral_error_ = lateral_error;
    heading_error_ = heading_error;
}

} // namespace planning
} // namespace mujianhua