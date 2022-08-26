#pragma once

#include <memory>
#include <vector>

#include <OsqpEigen/OsqpEigen.h>

#include "../../../planning/src/reference_line/reference_line.h"
#include "../../../planning/src/reference_line/reference_line_smoother.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "common/planning_dependency_injector.h"

namespace mujianhua {
namespace planning {

namespace {

struct DP_POINT;

} // namespace

class QPSplineReferenceLineSmoother : public ReferenceLineSmoother {
  public:
    QPSplineReferenceLineSmoother();

    QPSplineReferenceLineSmoother(
        std::shared_ptr<PlanningDependencyInjector> injector);

    bool Smooth(const std::vector<PathPoint> &raw_reference_points,
                ReferenceLine *reference_line, Frame *frame) override;

    const std::string Name() const override;

  private:
    std::string name_;
    std::shared_ptr<PlanningDependencyInjector> injector_;

    std::vector<double> x_list_, y_list_, s_list_;

    std::vector<double> dp_layers_s_list_;
    std::vector<std::pair<double, double>> layers_bounds_;
    double vehicle_l_wrt_smoothed_ref_;

    bool SplineInterpolation();

    bool SegmetRawReferencePoints(ReferenceLine *reference_line);

    bool DPGraphSearch(Frame *frame, ReferenceLine *reference_line);

    void DPCalculateCost(std::vector<std::vector<DP_POINT>> &samples,
                         int layer_index, int lateral_index);

    bool Smooth(ReferenceLine *reference_line);

    void SetPostHessianMatrix(Eigen::SparseMatrix<double> *matrix_h);

    void
    SetPostConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                            Eigen::VectorXd *lower_bound,
                            Eigen::VectorXd *upper_bound) const;
};

} // namespace planning
} // namespace mujianhua