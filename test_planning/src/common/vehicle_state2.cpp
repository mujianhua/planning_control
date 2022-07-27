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

double &VehicleState2::x() { return x_; }

double &VehicleState2::y() { return y_; }

double &VehicleState2::heading() { return heading_; }

} // namespace planning
} // namespace mujianhua