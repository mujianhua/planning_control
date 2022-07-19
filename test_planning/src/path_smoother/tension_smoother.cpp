#include "path_smoother/tension_smoother.h"

namespace mujianhua {
namespace planning {

TensionSmoother::TensionSmoother(
    const std::vector<TrajectoryPoint> &initial_points,
    const TrajectoryPoint &start_point)
    : ReferencePathSmoother(initial_points, start_point) {}

bool TensionSmoother::Smooth(ReferencePath *reference_path) {
    ROS_INFO("This is Tension Smoother smooth method.");
    return true;
}

} // namespace planning
} // namespace mujianhua