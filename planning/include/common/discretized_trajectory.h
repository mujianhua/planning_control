#pragma once

#include <vector>
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

class DiscretizedTrajectory {
  public:
    DiscretizedTrajectory() = default;

    DiscretizedTrajectory(
        const std::vector<TrajectoryPoint> &trajectory_points);

    const std::vector<TrajectoryPoint> *trajectory_points() const {
        return &trajectory_points_;
    }

  private:
    std::vector<TrajectoryPoint> trajectory_points_;
};

} // namespace planning
} // namespace mujianhua
