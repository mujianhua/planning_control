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
                    const TrajectoryPoint &start_point);

  private:
    bool Smooth(ReferencePath *reference_path) override;
};

} // namespace planning
} // namespace mujianhua