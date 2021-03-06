/**
 * @author mujianhua
 * @brief
 */

#include "test_control/vehicle_state_provider.h"

namespace mujianhua {
namespace control {

bool VehicleStateProvider::Update(
    const test_control::chassis_data *chassis_data) {
    data_ = chassis_data; // 此处若不是 VehicleStateProvider 的 shared_ptr
                          // 会异常, why???
    return true;
}

double VehicleStateProvider::x() const { return data_->Xo; }

double VehicleStateProvider::y() const { return data_->Yo; }

double VehicleStateProvider::theta() const { return data_->Theta / 180 * M_PI; }

double VehicleStateProvider::roll() const { return data_->Roll / 180 * M_PI; }

double VehicleStateProvider::pitch() const { return data_->Pitch / 180 * M_PI; }

double VehicleStateProvider::vx() const {
    // return data_->Vx;
    return data_->Vx / 3.6;
}

double VehicleStateProvider::vy() const { return data_->Vy / 3.6; }

double VehicleStateProvider::yawrate() const {
    return data_->Yawrate / 180 * M_PI;
}

double VehicleStateProvider::rollrate() const {
    return data_->Rollrate / 180 * M_PI;
}

} // namespace control
} // namespace mujianhua