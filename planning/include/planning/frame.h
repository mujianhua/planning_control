/**
 * @file frame.h
 * @brief planning dependency, processing reference line and obstacles.
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "math/polygon2d.h"
#include "planning/indexed_list.h"
#include "planning/reference_line.h"
#include "planning_config.h"
#include "vehicle_param.h"

namespace planning {

class Frame {
 public:
  using DynamicObstacle = std::vector<std::pair<double, math::Polygon2d>>;
  using StaticObstacle = math::Polygon2d;
  using IndexedDynamicObstacles = IndexedList<std::string, DynamicObstacle>;
  using IndexedStaticObstacle = IndexedList<std::string, StaticObstacle>;

  explicit Frame(const PlanningConfig &config) : config_(config) {}

  const ReferenceLine &reference_line() const { return reference_line_; }

  void SetReferenceLine(const ReferenceLine &reference);

  IndexedStaticObstacle &index_static_obstacles() {
    return index_static_obstacles_;
  }

  IndexedDynamicObstacles &index_dynamic_obstacles() {
    return index_dynamic_obstacles_;
  }

  void AddObstacle(const std::string &id, const DynamicObstacle &obs);

  void AddObstacle(const std::string &id, const StaticObstacle &obs);

  void ClearObstacles();

  void ClearStaticObstacles() { index_static_obstacles_.ClearAll(); }

  void ClearDynamicObstacles() { index_dynamic_obstacles_.ClearAll(); }

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

  // TODO:
  IndexedDynamicObstacles index_dynamic_obstacles_;
  IndexedStaticObstacle index_static_obstacles_;

  ReferenceLine reference_line_;
  std::vector<math::Vec2d> road_barrier_;
};

}  // namespace planning
