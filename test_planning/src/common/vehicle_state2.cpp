#include "common/vehicle_state2.h"

namespace mujianhua {
namespace planning {

VehicleState2::VehicleState2(double x, double y, double heading)
    : x_(x), y_(y), heading_(heading) {}

void VehicleState2::Update(const double &x, const double &y,
                           const double &heading) {
    x_ = x;
    y_ = y;
    heading_ = heading;
}

const double &VehicleState2::x() const { return x_; }

const double &VehicleState2::y() const { return y_; }

const double &VehicleState2::heading() const { return heading_; }

const double &VehicleState2::lateral_error() const { return lateral_error_; }

const double &VehicleState2::heading_error() const { return heading_error_; }

double &VehicleState2::x() { return x_; }

double &VehicleState2::y() { return y_; }

double &VehicleState2::heading() { return heading_; }

void VehicleState2::SetInitialError(double lateral_error,
                                    double heading_error) {
    lateral_error_ = lateral_error;
    heading_error_ = heading_error;
}

} // namespace planning
} // namespace mujianhua