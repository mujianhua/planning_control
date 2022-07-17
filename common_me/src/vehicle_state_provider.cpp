/**
 * @author mujianhua
 * @brief
 */

#include "common_me/vehicle_state_provider.h"

namespace mujianhua {
namespace common {

bool VehicleStateProvider::Update(const common_me::ChassisData *chassis_data) {
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

} // namespace common
} // namespace mujianhua