/**
 * @file
 * @brief
 */

#pragma once

#include <algorithm>
#include <ros/ros.h>
#include <vector>

#include "Eigen/Eigen"
#include "osqp/osqp.h"

namespace mujianhua {
namespace control {
namespace math {

class MpcOsqp {
  public:
    /**
     * @brief constructor
     */
    MpcOsqp(const Eigen::MatrixXd &matrix_a, const Eigen::MatrixXd &matrix_b,
            const Eigen::MatrixXd &matrix_q, const Eigen::MatrixXd &matrix_r,
            const Eigen::MatrixXd &matrix_initial_x,
            const Eigen::MatrixXd &matrix_u_lower,
            const Eigen::MatrixXd &matrix_u_upper,
            const Eigen::MatrixXd &matrix_x_lower,
            const Eigen::MatrixXd &matrix_x_upper,
            const Eigen::MatrixXd &matrix_x_ref, const int horizon,
            const int max_iter);

    /**
     * @brief 求解控制命令(front wheel angle)
     * @return 求解成功返回true.
     */
    bool Solve(std::vector<double> *control_cmd);

  private:
    /**
     * @brief
     */
    void CalculateGradient();

    /**
     * @brief
     */
    void CalculateConstraintVectors();

    /**
     * @brief 计算二次规划权重矩阵
     * @param
     */
    void CalculateKernel(std::vector<c_float> *P_data,
                         std::vector<c_int> *P_indices,
                         std::vector<c_int> *P_indptr);

    /**
     * @brief 等式约束. x(k+1) = Ax(k).
     * @param
     */
    void CalculateEqualityConstraint(std::vector<c_float> *A_data,
                                     std::vector<c_int> *A_indices,
                                     std::vector<c_int> *A_indptr);

    /**
     * @brief
     */
    OSQPData *Data();

    /**
     * @brief
     */
    OSQPSettings *Settings();

    /**
     * @brief
     */
    void FreeData(OSQPData *data);

    /**
     * @brief
     */
    template <typename T> T *CopyData(const std::vector<T> &vec) {
        T *data = new T[vec.size()];
        memcpy(data, vec.data(), sizeof(T) * vec.size());
        return data;
    }

    Eigen::MatrixXd matrix_a_;
    Eigen::MatrixXd matrix_b_;
    Eigen::MatrixXd matrix_q_;
    Eigen::MatrixXd matrix_r_;
    Eigen::MatrixXd matrix_initial_x_;
    const Eigen::MatrixXd matrix_u_lower_;
    const Eigen::MatrixXd matrix_u_upper_;
    const Eigen::MatrixXd matrix_x_lower_;
    const Eigen::MatrixXd matrix_x_upper_;
    const Eigen::MatrixXd matrix_x_ref_;
    Eigen::VectorXd lowerBound_;
    Eigen::VectorXd upperBound_;

    int max_iteration_;

    size_t horizon_;
    size_t state_dim_;
    size_t control_dim_;
    size_t num_param_;

    Eigen::VectorXd gradient_;
};

} // namespace math
} // namespace control
} // namespace mujianhua
