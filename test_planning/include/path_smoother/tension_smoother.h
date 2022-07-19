#pragma once

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <ros/ros.h>
#include "path_smoother/reference_path_smoother.h"

namespace mujianhua {
namespace planning {

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
};

} // namespace planning
} // namespace mujianhua