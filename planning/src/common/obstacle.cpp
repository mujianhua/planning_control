#include "common/obstacle.h"

namespace mujianhua {
namespace planning {
namespace common {

Obstacle::Obstacle(const std::string &id, const math::Polygon2d polygon2d,
                   const bool is_static)
    : id_(id), polygon2d_(polygon2d), is_static_(is_static) {}

} // namespace common
} // namespace planning
} // namespace mujianhua