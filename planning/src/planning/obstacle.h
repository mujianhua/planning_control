#pragma once

#include <string>
#include <vector>

#include "../math/polygon2d.h"

namespace planning {

class Obstacle {
 public:
  using DynamicObstacle = std::vector<std::pair<double, math::Polygon2d>>;
  using StaticObstacle = math::Polygon2d;

  Obstacle() = default;

  Obstacle(const std::string &id, const StaticObstacle &obs);

  Obstacle(const std::string &id, const DynamicObstacle &obs);

  const std::string &Id() const { return id_; }

  bool IsStatic() const { return is_static_; }

  const DynamicObstacle &polygon2ds() const { return dynamic_obs_; }

  const StaticObstacle &polygon2d() const { return static_obs_; }

 private:
  std::string id_;
  bool is_static_;

  DynamicObstacle dynamic_obs_;
  StaticObstacle static_obs_;
};

}  // namespace planning
