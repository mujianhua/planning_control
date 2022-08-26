#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include "../../../planning/src/reference_line/reference_line.h"
#include "common/frame.h"
#include "common/math/math_util.h"
#include "reference_line/reference_point.h"
#include "tinyspline_ros/tinysplinecpp.h"

namespace mujianhua {
namespace planning {

class ReferenceLineSmoother {
  public:
    ReferenceLineSmoother() = default;

    virtual bool Smooth(const std::vector<PathPoint> &raw_reference_points,
                        ReferenceLine *smoothed_reference_line,
                        Frame *frame) = 0;

    virtual const std::string Name() const = 0;

  protected:
    std::vector<PathPoint> ref_points_;
};

} // namespace planning
} // namespace mujianhua