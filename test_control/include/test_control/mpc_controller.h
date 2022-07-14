/**
 * @file
 * @brief Defines the MPC Controller class.
 */

#pragma once
#include <iostream>

#include "Eigen/Core"
#include "controller.h"
#include "math/mpc_osqp.h"
#include "trajectory_analyzer.h"

namespace mujianhua {
namespace control {

/**
 * @class MPCController
 *
 * @brief
 */
class MPCController : public Controller {
  public:
    /**
     * @brief constructor
     */
    MPCController();

    /**
     * @brief
     * @param
     */
    bool Init(const ControlConf *control_conf, const VehiclePara *vehicle_para,
              std::shared_ptr<VehicleStateProvider> vehicle_state) override;

    bool ComputeControlCommand(test_control::chassis_cmd *cmd,
                               test_control::simple_mpc_debug *debug,
                               const ADCTrajectory *trajectory) override;

    const std::string Name() const override;

  private:
    void UpdateState(test_control::simple_mpc_debug *debug);

    void UpdateMatrix(test_control::simple_mpc_debug *debug);

    void ComputeLateralErrors(const double x, const double y,
                              const double theta,
                              test_control::simple_mpc_debug *debug);

    bool LoadControlConf(const ControlConf *control_conf,
                         const VehiclePara *vehicle_para);

    const std::string name_;
    const ControlConf *control_conf_;

    TrajectoryAnalyzer trajectory_analyzer_;

    std::shared_ptr<VehicleStateProvider> vehicle_state_;

    double ts_ = 0.0;
    double g_ = 9.81;
    double cf_ = 0.0;
    double cr_ = 0.0;
    double wheelbase_ = 0.0;
    double mass_ = 0.0;
    double mass_s_ = 0.0;
    double lf_ = 0.0;
    double lr_ = 0.0;
    double iz_ = 0.0;
    double ix_ = 0.0;
    double steer_ratio_ = 0.0;
    double h_ = 0.0;
    double k_phi_ = 0.0;
    double d_phi_ = 0.0;

    int basic_state_size_ = 6;
    int controls_ = 2;
    int horizon_ = 10;

    double max_vy_;
    double max_roll_;
    double max_yawrate_;
    double max_rollrate_;
    double max_fwa_;

    // parameters for mpc solver; number of iterations
    int mpc_max_iteration_ = 0;

    Eigen::MatrixXd matrix_a_;
    Eigen::MatrixXd matrix_ad_;
    Eigen::MatrixXd matrix_a_coeff_;
    Eigen::MatrixXd matrix_b_;
    Eigen::MatrixXd matrix_bd_;
    Eigen::MatrixXd matrix_c_;
    Eigen::MatrixXd matrix_cd_;
    Eigen::MatrixXd matrix_state_;
    Eigen::MatrixXd matrix_q_;
    Eigen::MatrixXd matrix_r_;

    Eigen::MatrixXd lower_control_bound_;
    Eigen::MatrixXd upper_control_bound_;
    Eigen::MatrixXd lower_state_bound_;
    Eigen::MatrixXd upper_state_bound_;
};

} // namespace control
} // namespace mujianhua