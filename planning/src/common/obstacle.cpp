#include "common/obstacle.h"
#include "common/math/polygon2d.h"

namespace mujianhua {
namespace planning {

Obstacle::Obstacle(const std::string &id, const math::Polygon2d polygon2d,
                   const bool is_static)
    : id_(id), polygon2d_(polygon2d), is_static_(is_static) {}

Obstacle::Obstacle(const std::string &id, const DynamicObstacle polygon2ds,
                   const bool is_static)
    : id_(id), polygon2ds_(polygon2ds), is_static_(is_static) {}

} // namespace planning
} // namespace mujianhua
