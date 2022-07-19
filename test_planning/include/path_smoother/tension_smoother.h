#pragma once

#include <string>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <ros/ros.h>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "path_smoother/reference_path_smoother.h"

namespace mujianhua {
namespace planning {

using CppAD::AD;

class TensionSmoother : public ReferencePathSmoother {
  public:
    TensionSmoother() = delete;

    TensionSmoother(const std::vector<TrajectoryPoint> &initial_points,
                    const TrajectoryPoint &start_point, const Map &grid_map);

  private:
    bool Smooth(ReferencePath *reference_path) override;

    virtual bool IpoptSmooth(const std::vector<double> &x_list,
                             const std::vector<double> &y_list,
                             const std::vector<double> &s_list,
                             const std::vector<double> &heading_list,
                             const std::vector<double> &kappa_list,
                             std::vector<double> *result_x_list,
                             std::vector<double> *result_y_list,
                             std::vector<double> *result_s_list);

    virtual bool OsqpSmooth(const std::vector<double> &x_list,
                            const std::vector<double> &y_list,
                            const std::vector<double> &s_list,
                            const std::vector<double> &heading_list,
                            const std::vector<double> &kappa_list,
                            std::vector<double> *result_x_list,
                            std::vector<double> *result_y_list,
                            std::vector<double> *result_s_list);

    virtual void
    OsqpSetHessianMatrix(size_t size,
                         Eigen::SparseMatrix<double> *matrix_h) const;

    void OsqpSetConstraintMatrix(
        const std::vector<double> &x_list, const std::vector<double> &y_list,
        const std::vector<double> &s_list,
        const std::vector<double> &heading_list,
        const std::vector<double> &kappa_list,
        Eigen::SparseMatrix<double> *matrix_constraints,
        Eigen::VectorXd *lowerbound, Eigen::VectorXd *upperbound) const;
};

} // namespace planning
} // namespace mujianhua