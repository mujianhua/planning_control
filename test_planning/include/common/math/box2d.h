#pragma once

#include <limits>
#include <vector>
#include "common/math/vec2d.h"

namespace mujianhua {
namespace planning {
namespace math {

class Box2d {
  public:
    Box2d() = default;

    Box2d(const Vec2d &center, const double heading, const double length,
          const double width);

  private:
    Vec2d center_;
    double heading_ = 0.0;
    double length_ = 0.0;
    double width_ = 0.0;
    double half_length_ = 0.0;

    std::vector<Vec2d> corners_;

    double max_x_ = std::numeric_limits<double>::lowest();
    double min_x_ = std::numeric_limits<double>::max();
    double max_y_ = std::numeric_limits<double>::lowest();
    double min_y_ = std::numeric_limits<double>::max();
};

} // namespace math
} // namespace planning
} // namespace mujianhua