/**
 * @author mujianhua
 * @brief
 */

#include "math/mpc_osqp.h"

namespace mujianhua {
namespace control {
namespace math {

MpcOsqp::MpcOsqp(const Eigen::MatrixXd &matrix_a,
                 const Eigen::MatrixXd &matrix_b,
                 const Eigen::MatrixXd &matrix_q,
                 const Eigen::MatrixXd &matrix_r,
                 const Eigen::MatrixXd &matrix_initial_x,
                 const Eigen::MatrixXd &matrix_u_lower,
                 const Eigen::MatrixXd &matrix_u_upper,
                 const Eigen::MatrixXd &matrix_x_lower,
                 const Eigen::MatrixXd &matrix_x_upper,
                 const Eigen::MatrixXd &matrix_x_ref, const int horizon,
                 const int max_iter)
    : matrix_a_(matrix_a), matrix_b_(matrix_b), matrix_q_(matrix_q),
      matrix_r_(matrix_r), matrix_initial_x_(matrix_initial_x),
      matrix_u_lower_(matrix_u_lower), matrix_u_upper_(matrix_u_upper),
      matrix_x_lower_(matrix_x_lower), matrix_x_upper_(matrix_x_upper),
      matrix_x_ref_(matrix_x_ref), horizon_(horizon), max_iteration_(max_iter) {
    state_dim_ = matrix_b.rows();
    control_dim_ = matrix_b.cols();
    ROS_DEBUG("state_dim %zu", state_dim_);
    ROS_DEBUG("control_dim %zu", control_dim_);
    num_param_ = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
}

void MpcOsqp::CalculateGradient() {
    gradient_ = Eigen::VectorXd::Zero(
        state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    for (size_t i = 0; i < horizon_ + 1; i++) {
        gradient_.block(i * state_dim_, 0, state_dim_, 1) =
            -1.0 * matrix_q_ * matrix_x_ref_;
    }
}

void MpcOsqp::CalculateConstraintVectors() {
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(
        state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(
        state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    for (size_t i = 0; i < horizon_; i++) {
        lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                              control_dim_, 1) = matrix_u_lower_;
        upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                              control_dim_, 1) = matrix_u_upper_;
    }
    for (size_t i = 0; i < horizon_ + 1; i++) {
        lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) =
            matrix_x_lower_;
        upperInequality.block(state_dim_ * i, 0, state_dim_, 1) =
            matrix_x_upper_;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality =
        Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0, 0, state_dim_, 1) = -1.0 * matrix_initial_x_;
    upperEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound_ = Eigen::MatrixXd::Zero(
        2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    lowerBound_ << lowerEquality, lowerInequality;
    upperBound_ = Eigen::MatrixXd::Zero(
        2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
    upperBound_ << upperEquality, upperInequality;
}

void MpcOsqp::CalculateKernel(std::vector<c_float> *P_data,
                              std::vector<c_int> *P_indices,
                              std::vector<c_int> *P_indptr) {

    // col1:(row, val),...; col2:(row, val),...; ...
    std::vector<std::vector<std::pair<c_int, c_float>>> columns;
    columns.resize(num_param_);
    size_t value_index = 0;
    // state and terminal state
    for (size_t i = 0; i < horizon_ + 1; i++) {
        for (size_t j = 0; j < state_dim_; j++) {
            columns[i * state_dim_ + j].emplace_back(i * state_dim_ + j,
                                                     matrix_q_(j, j));
            ++value_index;
        }
    }
    // control
    const size_t state_total_dim = state_dim_ * (horizon_ + 1);
    for (size_t i = 0; i < horizon_; i++) {
        for (size_t j = 0; j < control_dim_; j++) {
            columns[i * control_dim_ + j + state_total_dim].emplace_back(
                i * control_dim_ + j, matrix_r_(j, j));
            ++value_index;
        }
    }

    int ind_p = 0;
    for (size_t i = 0; i < num_param_; i++) {
        P_indptr->emplace_back(ind_p);
        for (auto &row_data_pair : columns[i]) {
            P_data->emplace_back(row_data_pair.second);
            P_indices->emplace_back(row_data_pair.first);
            ++ind_p;
        }
    }
    P_indptr->emplace_back(ind_p);
}

void MpcOsqp::CalculateEqualityConstraint(std::vector<c_float> *A_data,
                                          std::vector<c_int> *A_indices,
                                          std::vector<c_int> *A_indptr) {
    static constexpr double kEpsilon = 1e-6;

    Eigen::MatrixXd matrix_constraint = Eigen::MatrixXd::Zero(
        state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
            control_dim_ * horizon_,
        state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);
    Eigen::MatrixXd state_identity_mat = Eigen::MatrixXd::Identity(
        state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1));

    matrix_constraint.block(0, 0, state_dim_ * (horizon_ + 1),
                            state_dim_ * (horizon_ + 1)) =
        -1 * state_identity_mat;

    for (size_t i = 0; i < horizon_; i++) {
        matrix_constraint.block(state_dim_ * (i + 1), state_dim_ * i,
                                state_dim_, state_dim_) = matrix_a_;
    }

    for (size_t i = 0; i < horizon_; i++) {
        matrix_constraint.block(state_dim_ * (i + 1),
                                control_dim_ * i + state_dim_ * (horizon_ + 1),
                                state_dim_, control_dim_) = matrix_b_;
    }

    Eigen::MatrixXd all_identity_mat =
        Eigen::MatrixXd::Identity(num_param_, num_param_);
    matrix_constraint.block(state_dim_ * (horizon_ + 1), 0, num_param_,
                            num_param_) = all_identity_mat;

    std::vector<std::vector<std::pair<c_int, c_float>>> columns;
    columns.resize(num_param_ + 1); // ???
    int value_index = 0;
    for (size_t i = 0; i < num_param_; i++) {
        for (size_t j = 0; j < num_param_ + state_dim_ * (horizon_ + 1); j++) {
            if (fabs(matrix_constraint(j, i)) > kEpsilon) {
                // (row, val)
                columns[i].emplace_back(j, matrix_constraint(j, i));
                ++value_index;
            }
        }
    }
    ROS_DEBUG("value_index: %d", value_index);
    int ind_A = 0;
    for (size_t i = 0; i < num_param_; i++) {
        A_indptr->emplace_back(ind_A);
        for (auto &row_data_pair : columns[i]) {
            A_data->emplace_back(row_data_pair.second);   // value
            A_indices->emplace_back(row_data_pair.first); // row
            ++ind_A;
        }
    }
    A_indptr->emplace_back(ind_A);
}

OSQPSettings *MpcOsqp::Settings() {
    OSQPSettings *settings =
        reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
    if (settings == nullptr) {
        return nullptr;
    } else {
        osqp_set_default_settings(settings);
        settings->max_iter = max_iteration_;
        return settings;
    }
}

OSQPData *MpcOsqp::Data() {
    OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));

    size_t kernel_dim = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
    size_t num_affine_constraint =
        2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
    if (data == nullptr) {
        return nullptr;
    } else {
        data->n = kernel_dim;            // nums of variables
        data->m = num_affine_constraint; // nums of constraints

        // the upper triangular part of the quadratic cost matrix P in csc
        // format
        std::vector<c_float> P_data;
        std::vector<c_int> P_indices;
        std::vector<c_int> P_indptr;
        ROS_DEBUG("before CalculateKernel.");
        CalculateKernel(&P_data, &P_indices, &P_indptr);
        data->P =
            csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                       CopyData(P_indices), CopyData(P_indptr));

        data->q = gradient_.data();

        // linear constraints matrix A in csc format
        std::vector<c_float> A_data;
        std::vector<c_int> A_indices;
        std::vector<c_int> A_indptr;
        CalculateEqualityConstraint(&A_data, &A_indices, &A_indptr);
        data->A = csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                             CopyData(A_data), CopyData(A_indices),
                             CopyData(A_indptr));

        data->l = lowerBound_.data();
        data->u = upperBound_.data();
        return data;
    }
}

void MpcOsqp::FreeData(OSQPData *data) {
    c_free(data->A);
    c_free(data->P);
    c_free(data);
}

bool MpcOsqp::Solve(std::vector<double> *control_cmd) {

    CalculateGradient();

    CalculateConstraintVectors();

    OSQPData *data = Data();
    ROS_DEBUG("OSQP data done");
    ROS_DEBUG("OSQP data n %lld ", data->n);
    ROS_DEBUG("OSQP data m %lld ", data->m);
    for (int i = 0; i < data->n; i++) {
        ROS_DEBUG("OSQP data q %d : %f", i, (data->q)[i]);
    }
    for (int i = 0; i < data->m; i++) {
        ROS_DEBUG("OSQP data l %d : %f", i, (data->l)[i]);
    }
    for (int i = 0; i < data->m; i++) {
        ROS_DEBUG("OSQP data u %d : %f", i, (data->u)[i]);
    }

    OSQPSettings *settings = Settings();
    ROS_DEBUG("OSQP setting done");

    OSQPWorkspace *osqp_workspace;

    c_int exitflag = 0;
    exitflag = osqp_setup(&osqp_workspace, data, settings);
    ROS_DEBUG("OSQP workspace ready");

    osqp_solve(osqp_workspace);

    auto status = osqp_workspace->info->status_val;
    ROS_DEBUG("status: %lld", status);
    // check status
    if (status < 0 || (status != 1 && status != 2)) {
        ROS_ERROR("failed optimization status:\t%s",
                  osqp_workspace->info->status);
        osqp_cleanup(osqp_workspace);
        FreeData(data);
        c_free(settings);
        return false;
    } else if (osqp_workspace->solution == nullptr) {
        ROS_ERROR("The solution from OSQP is nullptr");
        osqp_cleanup(osqp_workspace);
        FreeData(data);
        c_free(settings);
        return false;
    }
    size_t first_control = state_dim_ + (horizon_ + 1);
    for (size_t i = 0; i < control_dim_; ++i) {
        control_cmd->at(i) = osqp_workspace->solution->x[i + first_control];
        ROS_DEBUG("control_cmd: %lu : %f", i, control_cmd->at(i));
    }

    // control_cmd->at(0) = 0.1;

    // Cleanup
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return true;
}

} // namespace math
} // namespace control
} // namespace mujianhua
