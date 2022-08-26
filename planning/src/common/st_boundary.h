
#pragma once

#include <string>
#include <vector>

#include "../math/line_segment2d.h"
#include "../math/polygon2d.h"
#include "../math/vec2d.h"
#include "st_point.h"

namespace planning {

class STBoundary : public math::Polygon2d {
 public:
  STBoundary() = default;

  explicit STBoundary(
      const std::vector<std::pair<STPoint, STPoint>> &point_pairs,
      bool is_accurate_boundary = false);

  void set_bottom_left_point(STPoint st_point);
  void set_bottom_right_point(STPoint st_point);
  void set_upper_left_point(STPoint st_point);
  void set_upper_right_point(STPoint st_point);

 private:
  bool IsPointNear(const math::LineSegment2d &seg, const math::Vec2d &point,
                   const double max_dist);

  void RemoveRedundantPoints(
      const std::vector<std::pair<STPoint, STPoint>> *point_pairs);

  std::string id_;

  // (s,t)...
  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;

  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;
};

}  // namespace planning
