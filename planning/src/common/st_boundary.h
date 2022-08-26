#pragma once

#include <string>
#include <vector>

#include "st_point.h"
#include "../math/polygon2d.h"

namespace planning {

class STBoundary : public math::Polygon2d {
 public:
  STBoundary() = default;

  explicit STBoundary(
      const std::vector<std::pair<STPoint, STPoint>> &point_pairs,
      bool is_accurate_boundary = false);

 private:
  std::string id_;
  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;

  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;
};

}  // namespace planning
