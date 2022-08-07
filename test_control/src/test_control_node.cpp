/**
 * @author mujianhua
 * @brief
 */

#include <ros/ros.h>

#include "osqp/osqp.h"
#include "test_control/chassis_cmd.h"
#include "test_control/chassis_data.h"
#include "test_control/controller.h"
#include "test_control/mpc_controller.h"
#include "test_control/simple_mpc_debug.h"
#include "test_control/vehicle_state_provider.h"

using namespace mujianhua::control;

class ControlNode {
  public:
    explicit ControlNode(const ros::NodeHandle &nh) : nh_(nh) {
        controller_ = std::make_shared<MPCController>();
        vehicle_state_ = std::make_shared<VehicleStateProvider>();
        ROS_INFO("The controller is [%s]!", controller_->Name().c_str());

        ControlConf control_conf{};
        VehiclePara vehicle_para{};
        LoadConfig(control_conf, vehicle_para);

        controller_->Init(&control_conf, &vehicle_para, vehicle_state_);

        debug_pub_ =
            nh_.advertise<test_control::simple_mpc_debug>("mpc_debug", 1);
        chassis_pub_ = nh_.advertise<test_control::chassis_cmd>("control", 1);

        chassis_sub_ = nh_.subscribe("/chassis_data", 1,
                                     &ControlNode::ReceiveChassisDataCb, this);
        trajectory = new DiscretizedTrajectory();
    }

    void ReceiveChassisDataCb(const test_control::chassis_data &msg) {
        test_control::chassis_cmd cmd;
        test_control::simple_mpc_debug debug;

        // Update vehicle state from carsim.
        vehicle_state_->Update(&msg); // why??????????

        double sleep_time =
            msg.simulink_time - (time_cache_ + 1 / controller_frequency_);
        if (sleep_time > 0.0) {
            time_cache_ = msg.simulink_time;
            controller_->ComputeControlCommand(&cmd, &debug, trajectory);
            chassis_pub_.publish(cmd);
            debug_pub_.publish(debug);
        }
    }

    void LoadConfig(ControlConf &control_conf, VehiclePara &vehicle_para) {
        nh_.param("test_control/ControlConfig/ts", control_conf.ts, 0.0);
        nh_.param("test_control/ControlConfig/horizon", control_conf.horizon,
                  0);
        nh_.param("test_control/ControlConfig/state_size",
                  control_conf.state_size, 0);
        nh_.param("test_control/ControlConfig/control_size",
                  control_conf.control_size, 0);

        nh_.param("test_control/ControlConfig/max_vy", control_conf.max_vy,
                  0.0);
        nh_.param("test_control/ControlConfig/max_yawrate",
                  control_conf.max_yawrate, 0.0);
        nh_.param("test_control/ControlConfig/max_roll", control_conf.max_roll,
                  0.0);
        nh_.param("test_control/ControlConfig/max_rollrate",
                  control_conf.max_rollrate, 0.0);
        nh_.param("test_control/ControlConfig/max_fwa",
                  control_conf.max_fwa_deg, 0.0);

        nh_.param("test_control/ControlConfig/mpc_max_iteration",
                  control_conf.mpc_max_iteration, 0);
        std::vector<int> matrix_q(control_conf.state_size);
        std::vector<int> matrix_r(control_conf.control_size);
        nh_.param("test_control/ControlConfig/mpc_matrix_q_vy", matrix_q[0], 0);
        nh_.param("test_control/ControlConfig/mpc_matrix_q_yawrate",
                  matrix_q[1], 0);
        nh_.param("test_control/ControlConfig/mpc_matrix_q_rollrate",
                  matrix_q[2], 0);
        nh_.param("test_control/ControlConfig/mpc_matrix_q_roll", matrix_q[3],
                  0);
        nh_.param("test_control/ControlConfig/mpc_matrix_q_ey", matrix_q[4], 0);
        nh_.param("test_control/ControlConfig/mpc_matrix_q_epsi", matrix_q[5],
                  0);
        nh_.param("test_control/ControlConfig/mpc_matrix_r_fwa", matrix_r[0],
                  0);
        control_conf.mpc_matrix_q = matrix_q;
        control_conf.mpc_matrix_r = matrix_r;

        control_conf.cf = -90000.0;
        control_conf.cr = -90000.0;

        nh_.param("test_control/VehicleParam/mass", vehicle_para.mass, 0.0);
        nh_.param("test_control/VehicleParam/mass_s", vehicle_para.mass_s, 0.0);
        nh_.param("test_control/VehicleParam/iz", vehicle_para.iz, 0.0);
        nh_.param("test_control/VehicleParam/ix", vehicle_para.ix, 0.0);
        nh_.param("test_control/VehicleParam/wheelbase", vehicle_para.wheelbase,
                  0.0);
        nh_.param("test_control/VehicleParam/steer_ratio",
                  vehicle_para.steer_ratio, 0.0);
        nh_.param("test_control/VehicleParam/lf", vehicle_para.lf, 0.0);
        nh_.param("test_control/VehicleParam/lr", vehicle_para.lr, 0.0);
        nh_.param("test_control/VehicleParam/h", vehicle_para.h, 0.0);
        nh_.param("test_control/VehicleParam/k_phi", vehicle_para.k_phi, 0.0);
        nh_.param("test_control/VehicleParam/d_phi", vehicle_para.d_phi, 0.0);
    }

  private:
    ros::NodeHandle nh_;
    // publish control command (steer_sw, ...)
    ros::Publisher chassis_pub_;
    // publish debug information (later error, ...)
    ros::Publisher debug_pub_;
    // subscribe chassis information (x, y, vx, vy, ...)
    ros::Subscriber chassis_sub_;
    std::shared_ptr<Controller> controller_;
    // packing chassis information and location information,
    std::shared_ptr<VehicleStateProvider>
        vehicle_state_; // shared_ptr nb... why?
    // simulation time cache, Ensure that the controller is allowed at 100Hz
    // frequency
    double time_cache_ = 0.0;
    double controller_frequency_ = 100.0;
    DiscretizedTrajectory *trajectory;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_control");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    //                                ros::console::levels::Debug);
    ros::NodeHandle nh;
    ControlNode node(nh);
    ros::spin();
    return 0;
}
