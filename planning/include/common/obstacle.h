#pragma once

#include <string>
#include "common/math/polygon2d.h"

namespace mujianhua {
namespace planning {
namespace common {

class Obstacle {
  public:
    Obstacle() = default;

    Obstacle(const std::string &id, const math::Polygon2d polygon2d,
             const bool is_static);

  private:
    std::string id_;
    bool is_static_;
    math::Polygon2d polygon2d_;
};

} // namespace common
} // namespace planning
} // namespace mujianhua