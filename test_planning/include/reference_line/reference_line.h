#pragma once

#include <vector>
#include ""
#include "reference_line/reference_point.h"

namespace mujianhua {
namespace planning {

class ReferenceLine {
  public:
    ReferenceLine() = default;

    explicit ReferenceLine(const std::vector<ReferencePoint> &reference_points);

  private:
    std::vector<ReferencePoint> reference_points_;
};

} // namespace planning
} // namespace mujianhua
