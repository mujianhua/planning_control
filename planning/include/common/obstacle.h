#pragma once

#include <string>
#include "common/indexed_list.h"
#include "common/math/polygon2d.h"

namespace mujianhua {
namespace planning {
namespace common {

using DynamicObstacle = std::vector<std::pair<double, math::Polygon2d>>;

class Obstacle {
  public:
    Obstacle() = default;

    Obstacle(const std::string &id, const math::Polygon2d polygon2d,
             const bool is_static);

    Obstacle(const std::string &id, const DynamicObstacle polygon2ds,
             const bool is_static);

    const std::string &Id() const { return id_; }

    const math::Polygon2d &PerceptionPolygon() const { return polygon2d_; }

    inline bool IsStatic() const { return is_static_; }

  private:
    std::string id_;
    bool is_static_;

    math::Polygon2d polygon2d_;
    std::vector<std::pair<double, math::Polygon2d>> polygon2ds_;
};

using IndexedObstacles = IndexedList<std::string, Obstacle>;
using ThreadSafeIndexedObstacles = ThreadSafeIndexedList<std::string, Obstacle>;

} // namespace common
} // namespace planning
} // namespace mujianhua