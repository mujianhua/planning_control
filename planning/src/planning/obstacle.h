/**
 * \file obstacle.h
 * \brief describe obstacle.
 * \author Jianhua Mu
 * \date 2022.08.31
 */

#pragma once

#include <string>
#include <vector>

#include "math/polygon2d.h"
#include "math/pose.h"

namespace planning {

/**
 * \class Obstacle
 */
class Obstacle {
 public:
  using DynamicObstacle =
      std::vector<std::pair<double, math::Polygon2d>>;  // (time, polygon2d)...
  using StaticObstacle = math::Polygon2d;
  using obstacle_trajectory =
      std::vector<std::pair<double, math::Pose>>;  // (time, pose)...

  Obstacle() = default;

  /**
   * \brief static obstacle constructor
   */
  Obstacle(const std::string &id, const StaticObstacle &obs);

  /**
   * \brief dynamic obstacle constructor
   * todo: delete...
   */
  Obstacle(const std::string &id, const DynamicObstacle &obs);

  /**
   * \brief dynamic obstacle constructor
   */
  Obstacle(const std::string &id, const math::Polygon2d &appearance,
           const obstacle_trajectory &traj);

  const std::string &Id() const { return id_; }

  bool IsStatic() const { return is_static_; }

  /**
   * \brief dynamic obstacles methods.
   */
  const DynamicObstacle &polygon2ds() const { return dynamic_obs_; }

  math::Polygon2d GetPolygon2dAtTime(const double relvative_time) const;

  /**
   * \brief static obstacles methods.
   */
  const StaticObstacle &polygon2d() const { return static_obs_; }

 private:
  std::string id_;
  bool is_static_;

  math::Polygon2d appearance_;

  DynamicObstacle dynamic_obs_;
  StaticObstacle static_obs_;
};

}  // namespace planning
