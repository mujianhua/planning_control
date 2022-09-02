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
  using obstacle_trajectory =
      std::vector<std::pair<double, math::Pose>>;  // (time, pose)...

  Obstacle() = default;

  /**
   * \brief static obstacle constructor
   */
  Obstacle(const std::string &id, const math::Polygon2d &appearance);

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

  math::Pose GetPoseAtTime(double relative_time) const;

  math::Polygon2d GetPolygonAtTime(double relative_time) const;

  /**
   * \brief static obstacles methods.
   */
  const math::Polygon2d &polygon2d() const { return appearance_; }

 private:
  std::string id_;
  bool is_static_{};

  math::Polygon2d appearance_;
  obstacle_trajectory trajectory_;

  DynamicObstacle dynamic_obs_;
};

}  // namespace planning
