#include "common/math/box2d.h"

namespace mujianhua {
namespace planning {
namespace math {

Box2d::Box2d(const Vec2d &center, const double heading, const double length,
             const double width)
    : center_(center), heading_(heading), length_(length), width_(width) {}

} // namespace math
} // namespace planning
} // namespace mujianhua