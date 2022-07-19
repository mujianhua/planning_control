#pragma once

#include <grid_map_core/grid_map_core.hpp>
#include <ros/ros.h>
#include "Eigen/Core"

namespace mujianhua {
namespace planning {

class Map {
  public:
    Map() = delete;

    explicit Map(const grid_map::GridMap &grid_map);

    double GetObstacleDistance(const Eigen::Vector2d &pos) const;

    bool IsInside(const Eigen::Vector2d &pos) const;

  private:
    const grid_map::GridMap map_;
};

} // namespace planning
} // namespace mujianhua
