#pragma once

#include <memory>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "common/planning_dependency_injector.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_line_smoother.h"

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

    bool Smooth(std::vector<ReferencePoint> &raw_reference_points,
                ReferenceLine *smoothed_reference_line, Frame *frame) override;

    const std::string Name() const override;

  private:
    std::string name_;
    std::shared_ptr<PlanningDependencyInjector> injector_;

    std::vector<double> x_list_, y_list_, s_list_;

    std::vector<double> dp_layers_s_list_;
    std::vector<std::pair<double, double>> layers_bounds_;

    bool SplineInterpolation();

    bool SegmetRawReferencePoints(std::vector<double> *x_list,
                                  std::vector<double> *y_list,
                                  std::vector<double> *s_list,
                                  std::vector<double> *theta_list,
                                  std::vector<double> *kappa_list);

    bool DPGraphSearch(Frame *frame, ReferenceLine *reference_line);

    void DPCalculateCost(std::vector<std::vector<DP_POINT>> &samples,
                         int layer_index, int lateral_index);

    bool Smooth(ReferenceLine *reference_line);
};

} // namespace planning
} // namespace mujianhua