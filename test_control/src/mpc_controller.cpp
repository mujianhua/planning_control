/**
 * @author mujianhua
 * @brief
 */

#include "test_control/mpc_controller.h"

#include "test_control/controller.h"

namespace mujianhua {
namespace control {

using Matrix = Eigen::MatrixXd;

MPCController::MPCController() : name_("MPC Controller") {}

bool MPCController::LoadControlConf(const ControlConf *control_conf,
                                    const VehiclePara *vehicle_para) {
    if (!control_conf) {
        ROS_ERROR("[MPCController] control_conf = nullptr");
        return false;
    }
    ts_ = control_conf->ts;
    if (ts_ <= 0.0) {
        ROS_ERROR("[MPCController] Invalid control update interval.");
        return false;
    }
    ts_ = control_conf->ts;
    horizon_ = control_conf->horizon;
    controls_ = control_conf->control_size;
    basic_state_size_ = control_conf->state_size;
    cf_ = control_conf->cf;
    cr_ = control_conf->cr;

    mpc_max_iteration_ = control_conf->mpc_max_iteration;

    max_vy_ = control_conf->max_vy;
    max_yawrate_ = control_conf->max_yawrate;
    max_rollrate_ = control_conf->max_rollrate;
    max_roll_ = control_conf->max_roll;
    max_fwa_ = control_conf->max_fwa_deg / 180.0 * M_PI;

    mass_ = vehicle_para->mass;
    mass_s_ = vehicle_para->mass_s;
    wheelbase_ = vehicle_para->wheelbase;
    lf_ = vehicle_para->lf;
    lr_ = vehicle_para->lr;
    h_ = vehicle_para->h;
    iz_ = vehicle_para->iz;
    ix_ = vehicle_para->ix;
    steer_ratio_ = vehicle_para->steer_ratio;
    k_phi_ = vehicle_para->k_phi;
    d_phi_ = vehicle_para->d_phi;

    return true;
}

bool MPCController::Init(const ControlConf *control_conf,
                         const VehiclePara *vehicle_para,
                         std::shared_ptr<VehicleStateProvider> vehicle_state) {
    if (!LoadControlConf(control_conf, vehicle_para)) {
        ROS_ERROR("failed to load control conf");
        return false;
    }
    vehicle_state_ = vehicle_state;
    // 没有实时规划的轨迹跟踪,轨迹分析器暂时放到此处.

    // Matrix init operations.
    matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_a_(0, 2) = -(d_phi_ * h_) / ix_;
    matrix_a_(0, 3) = -(ix_ * g_ + k_phi_ * h_) / ix_;
    matrix_a_(2, 2) = -d_phi_ / ix_;
    matrix_a_(2, 3) = -k_phi_ / ix_;
    matrix_a_(3, 2) = 1;
    matrix_a_(4, 0) = 1;
    matrix_a_(5, 1) = 1;

    matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_a_coeff_(0, 0) =
        (mass_ * h_ * h_ + ix_) * (cf_ + cr_) / (ix_ * mass_);
    matrix_a_coeff_(1, 0) = (cf_ * lf_ - cr_ * lr_) / iz_;
    matrix_a_coeff_(1, 1) = (cf_ * lf_ * lf_ + cr_ * lr_ * lr_) / iz_;
    matrix_a_coeff_(2, 0) = (h_ * (cf_ + cr_)) / (ix_);
    matrix_a_coeff_(2, 1) = (h_ * (cf_ * lf_ - cr_ * lr_)) / (ix_);

    matrix_b_ = Matrix::Zero(basic_state_size_, controls_);
    matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
    matrix_b_(0, 0) = -(cf_ * (mass_ * h_ * h_ + ix_)) / (ix_ * mass_);
    matrix_b_(1, 0) = -(cf_ * lf_) / iz_;
    matrix_b_(2, 0) = -(cf_ * h_) / ix_;
    matrix_bd_ = matrix_b_ * ts_;

    matrix_c_ = Matrix::Zero(basic_state_size_, 1);
    matrix_cd_ = Matrix::Zero(basic_state_size_, 1);

    matrix_state_ = Matrix::Zero(basic_state_size_, 1);

    matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    int q_param_size = control_conf->mpc_matrix_q.size();
    for (int i = 0; i < q_param_size; i++) {
        matrix_q_(i, i) = control_conf->mpc_matrix_q[i];
    }

    matrix_r_ = Matrix::Zero(controls_, controls_);
    int r_param_size = control_conf->mpc_matrix_r.size();
    for (int i = 0; i < r_param_size; i++) {
        matrix_r_(i, i) = control_conf->mpc_matrix_r[i];
    }

    lower_control_bound_ = Matrix::Zero(controls_, 1);
    upper_control_bound_ = Matrix::Zero(controls_, 1);
    lower_state_bound_ = Matrix::Zero(basic_state_size_, 1);
    upper_state_bound_ = Matrix::Zero(basic_state_size_, 1);

    lower_control_bound_ << -max_fwa_;
    upper_control_bound_ << max_fwa_;
    lower_state_bound_ << -max_vy_, -max_yawrate_, -max_rollrate_, -max_roll_,
        -2.0, -2.0;
    upper_state_bound_ << max_vy_, max_yawrate_, max_rollrate_, max_roll_, 2.0,
        2.0;

    ROS_INFO("[MPCController] init done!");
    return true;
}

bool MPCController::ComputeControlCommand(
    test_control::chassis_cmd *cmd, test_control::simple_mpc_debug *debug,
    const DiscretizedTrajectory *trajectory) {
    // 有实时的规划轨迹时,轨迹分析器放到此处...
    trajectory_analyzer_ = std::move(TrajectoryAnalyzer(trajectory));

    UpdateState(debug);

    UpdateMatrix(debug);

    Matrix control_matrix = Matrix::Zero(controls_, 1);
    std::vector<Matrix> control(horizon_, control_matrix);

    Matrix reference_state = Matrix::Zero(basic_state_size_, 1);

    math::MpcOsqp mpc_osqp(
        matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, matrix_state_,
        lower_control_bound_, upper_control_bound_, lower_state_bound_,
        upper_state_bound_, reference_state, horizon_, mpc_max_iteration_);

    std::vector<double> control_cmd(controls_, 0);
    if (!mpc_osqp.Solve(&control_cmd)) {
        ROS_ERROR("MPC OSQP solver failed!");
    } else {
        ROS_DEBUG("MPC OSQP problem solved!");
        control[0](0, 0) = control_cmd.at(0);
    }
    cmd->Ctrl_SW =
        control[0](0, 0) * 180 / M_PI * steer_ratio_; // todo: 可能会有错误
    return true;
}

void MPCController::UpdateState(test_control::simple_mpc_debug *debug) {
    ComputeLateralErrors(vehicle_state_->x(), vehicle_state_->y(),
                         vehicle_state_->theta(), debug);
    // State matrix update.
    matrix_state_(0, 0) = vehicle_state_->vy();
    matrix_state_(1, 0) = vehicle_state_->yawrate();
    matrix_state_(2, 0) = vehicle_state_->rollrate();
    matrix_state_(3, 0) = vehicle_state_->roll();
    matrix_state_(4, 0) = debug->lateral_error;
    matrix_state_(5, 0) = debug->heading_error;

    ROS_DEBUG("[MPCController] Now the state is [%f, %f, %f, %f, %f, %f]",
              matrix_state_(0, 0), matrix_state_(1, 0), matrix_state_(2, 0),
              matrix_state_(3, 0), matrix_state_(4, 0), matrix_state_(5, 0));
}

void MPCController::ComputeLateralErrors(
    const double x, const double y, const double theta,
    test_control::simple_mpc_debug *debug) {
    const auto match_point =
        trajectory_analyzer_.QueryNearestPointByPosition(x, y, debug);
    const double delta_theta = match_point.theta - theta; // TODO: 有问题...
    debug->ref_heading = match_point.theta;
    debug->heading_error = delta_theta;
}

void MPCController::UpdateMatrix(test_control::simple_mpc_debug *debug) {
    const double v = vehicle_state_->vx();
    matrix_a_(0, 0) = matrix_a_coeff_(0, 0) / v;
    matrix_a_(0, 1) =
        -(cf_ * ix_ * lr_ - cf_ * ix_ * lf_ + ix_ * mass_ * v * v -
          cf_ * mass_ * h_ * h_ * lf_ + cr_ * mass_ * h_ * h_ * lr_) /
        (ix_ * mass_ * v);
    matrix_a_(1, 0) = matrix_a_coeff_(1, 0) / v;
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
    matrix_a_(2, 0) = matrix_a_coeff_(2, 0) / v;
    matrix_a_(2, 1) = matrix_a_coeff_(2, 1) / v;
    matrix_a_(4, 5) = v;

    Matrix matrix_i = Matrix::Identity(matrix_a_.rows(), matrix_a_.cols());
    matrix_ad_ = matrix_a_ * ts_ + matrix_i; // todo:
}

const std::string MPCController::Name() const { return name_; }

} // namespace control
} // namespace mujianhua
