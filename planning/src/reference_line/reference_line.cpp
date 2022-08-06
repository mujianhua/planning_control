#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

ReferenceLine::ReferenceLine(const std::vector<TrajectoryPoint> &points)
    : reference_points_(points) {}

} // namespace planning
} // namespace mujianhua