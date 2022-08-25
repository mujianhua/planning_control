/**
 * @file frame.h
 * @brief planning dependency, processing reference line and obstacles.
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "math/polygon2d.h"
#include "planning/data_struct.h"
#include "planning/indexed_list.h"
#include "planning/local_view.h"
#include "planning/obstacle.h"
#include "planning/reference_line.h"
#include "planning_config.h"
#include "vehicle_param.h"

namespace planning {

class Frame {
 public:
  using DynamicObstacle = std::vector<std::pair<double, math::Polygon2d>>;
  using StaticObstacle = math::Polygon2d;

  explicit Frame(const PlanningConfig &config) : config_(config) {}

  void Update(const LocalView &local_view);

  const ReferenceLine &reference_line() const { return reference_line_; }

  void SetReferenceLine(const ReferenceLine &reference);

  void SetVehicleState(const VehicleState &state) { vehicle_state_ = state; }

  const VehicleState &vehicle_state() const { return vehicle_state_; }

  const std::vector<const Obstacle *> &dynamic_obstacles() {
    return dynamic_obstacles_;
  }

  bool CheckCollision(double time, const math::Box2d &rect);

  bool CheckOptimizationCollision(double time, const math::Pose &pose,
                                  double collision_buffer = 0.0);

  std::unordered_map<int, math::Polygon2d> QueryDynamicObstacles(double time);

  void Visualize();

 private:
  bool CheckStaticCollision(const math::Box2d &rect);

  bool CheckDynamicCollision(double time, const math::Box2d &rect);

 private:
  PlanningConfig config_;

  std::vector<const Obstacle *> obstacles_;
  std::vector<const Obstacle *> static_obstacles_;
  std::vector<const Obstacle *> dynamic_obstacles_;

  ReferenceLine reference_line_;
  std::vector<math::Vec2d> road_barrier_;

  VehicleState vehicle_state_;
};

}  // namespace planning
