#include "math/math_util.h"

namespace mujianhua {
namespace planning {
namespace math {

double Distance(const TrajectoryPoint &p1, const TrajectoryPoint &p2) {
    return sqrt(pow(p1.path_point.x - p2.path_point.x, 2) +
                pow(p1.path_point.y - p2.path_point.y, 2));
}

} // namespace math
} // namespace planning
} // namespace mujianhua