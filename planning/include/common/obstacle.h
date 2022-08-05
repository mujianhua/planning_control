#pragma once

#include <string>
#include "common/math/polygon2d.h"

namespace mujianhua {
namespace planning {
namespace common {

class Obstacle {
  public:
    Obstacle() = default;

    Obstacle(const std::string &id, const bool is_static);

  private:
};

} // namespace common
} // namespace planning
} // namespace mujianhua