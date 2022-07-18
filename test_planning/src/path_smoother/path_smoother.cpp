#include "path_smoother/path_smoother.h"

namespace mujianhua {
namespace planning {

PathSmoother::PathSmoother(std::string &type,
                           const std::vector<TrajectoryPoint> &input_points,
                           const TrajectoryPoint &start_point)
    : input_points_(input_points), start_point_(start_point) {}

} // namespace planning
} // namespace mujianhua