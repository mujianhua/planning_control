/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source
 *codes. Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#pragma once

#include <memory>
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

    explicit Frame(const CartesianPlannerConfig &config) : config_(config) {}

    std::vector<math::Polygon2d> &obstacles() { return static_obstacles_; }

    std::vector<DynamicObstacle> &dynamic_obstacles() {
        return dynamic_obstacles_;
    }

    IndexedStaticObstacle &index_static_obstacles() {
        return index_static_obstacles_;
    }

    IndexedDynamicObstacles &index_dynamic_obstacles() {
        return index_dynamic_obstacles_;
    }

    const ReferenceLine &reference_line() const { return reference_line_; }

    void SetReferenceLine(const ReferenceLine &reference);

    bool CheckCollision(double time, const math::Box2d &rect);

    bool CheckOptimizationCollision(double time, const math::Pose &pose,
                                    double collision_buffer = 0.0);

    std::unordered_map<int, math::Polygon2d> QueryDynamicObstacles(double time);

    void Visualize();

  private:
    CartesianPlannerConfig config_;
    std::vector<DynamicObstacle> dynamic_obstacles_;
    std::vector<StaticObstacle> static_obstacles_;

    IndexedDynamicObstacles index_dynamic_obstacles_;
    IndexedStaticObstacle index_static_obstacles_;

    ReferenceLine reference_line_;
    std::vector<math::Vec2d> road_barrier_;

    bool CheckStaticCollision(const math::Box2d &rect);

    bool CheckDynamicCollision(double time, const math::Box2d &rect);
};

} // namespace planning
