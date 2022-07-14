#pragma once
#include <ros/ros.h>

#include <string>

#include "ADCTrajectory.h"
#include "control_conf.h"
#include "test_control/chassis_cmd.h"
#include "test_control/chassis_data.h"
#include "test_control/simple_mpc_debug.h"
#include "vehicle_state_provider.h"

namespace mujianhua {
namespace control {

class Controller {
  public:
    Controller() = default;

    virtual bool Init(const ControlConf *control_conf,
                      const VehiclePara *vehicle_para,
                      std::shared_ptr<VehicleStateProvider> vehicle_state) = 0;

    virtual bool ComputeControlCommand(test_control::chassis_cmd *cmd,
                                       test_control::simple_mpc_debug *debug,
                                       const ADCTrajectory *trajectory) = 0;

    virtual const std::string Name() const = 0;
};

} // namespace control
} // namespace mujianhua