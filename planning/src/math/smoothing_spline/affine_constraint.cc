
#include "math/smoothing_spline/affine_constraint.h"

#include <ros/ros.h>
#include "glog/logging.h"

namespace planning {

AffineConstraint::AffineConstraint(const bool is_equality)
    : is_equality_(is_equality) {}

AffineConstraint::AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                                   const Eigen::MatrixXd& constraint_boundary,
                                   const bool is_equality)
    : constraint_matrix_(constraint_matrix),
      constraint_boundary_(constraint_boundary),
      is_equality_(is_equality) {
  CHECK_EQ(constraint_boundary.rows(), constraint_matrix.rows());
}

void AffineConstraint::SetIsEquality(const double is_equality) {
  is_equality_ = is_equality;
}

const Eigen::MatrixXd& AffineConstraint::constraint_matrix() const {
  return constraint_matrix_;
}

const Eigen::MatrixXd& AffineConstraint::constraint_boundary() const {
  return constraint_boundary_;
}

bool AffineConstraint::AddConstraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {
  if (constraint_matrix.rows() != constraint_boundary.rows()) {
    ROS_ERROR(
        "Fail to add constraint because constraint matrix rows != constraint "
        "boundary rows.");
    ROS_ERROR("constraint matrix rows = %ld", constraint_matrix.rows());
    ROS_ERROR("constraint boundary rows = %ld", constraint_boundary.rows());
    return false;
  }

  if (constraint_matrix_.rows() == 0) {
    constraint_matrix_ = constraint_matrix;
    constraint_boundary_ = constraint_boundary;
    return true;
  }
  if (constraint_matrix_.cols() != constraint_matrix.cols()) {
    ROS_ERROR(
        "constraint_matrix_ cols and constraint_matrix cols do not match.");
    ROS_ERROR("constraint_matrix_.cols()= %ld", constraint_matrix_.cols());
    ROS_ERROR("constraint_matrix.cols() = %ld", constraint_matrix.cols());
    return false;
  }
  if (constraint_boundary.cols() != 1) {
    ROS_ERROR("constraint_boundary.cols() should be 1.");
    return false;
  }

  Eigen::MatrixXd n_matrix(constraint_matrix_.rows() + constraint_matrix.rows(),
                           constraint_matrix_.cols());
  Eigen::MatrixXd n_boundary(
      constraint_boundary_.rows() + constraint_boundary.rows(), 1);

  n_matrix << constraint_matrix_, constraint_matrix;
  n_boundary << constraint_boundary_, constraint_boundary;
  constraint_matrix_ = n_matrix;
  constraint_boundary_ = n_boundary;
  return true;
}

}  // namespace planning
