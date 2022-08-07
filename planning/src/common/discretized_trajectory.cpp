#include "common/discretized_trajectory.h"
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint> &trajectory_points)
    : trajectory_points_(trajectory_points) {}

} // namespace planning
} // namespace mujianhua
