#include "math/math_util.h"

namespace mujianhua {
namespace planning {
namespace math {

double Distance(const TrajectoryPoint &point1, const TrajectoryPoint &point2) {
    return sqrt(pow(point1.path_point.x - point2.path_point.x, 2) +
                pow(point1.path_point.y - point2.path_point.y, 2));
}

} // namespace math
} // namespace planning
} // namespace mujianhua