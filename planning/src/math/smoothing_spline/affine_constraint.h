/**
 * \brief constraint for quadprog programming. A*x = b, A*x <= b
 * matrix cols is optimize variable size.
 * matrix rows is constraints size.
 **/

#pragma once

#include "Eigen/Core"
#include "math/polynomial_xd.h"

namespace planning {

class AffineConstraint {
 public:
  AffineConstraint() = default;
  explicit AffineConstraint(const bool is_equality);
  explicit AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                            const Eigen::MatrixXd& constraint_boundary,
                            const bool is_equality);

  void SetIsEquality(const double is_equality);

  const Eigen::MatrixXd& constraint_matrix() const;
  const Eigen::MatrixXd& constraint_boundary() const;
  bool AddConstraint(const Eigen::MatrixXd& constraint_matrix,
                     const Eigen::MatrixXd& constraint_boundary);

 private:
  Eigen::MatrixXd constraint_matrix_;    // A
  Eigen::MatrixXd constraint_boundary_;  // b
  bool is_equality_ = true;
};

}  // namespace planning
