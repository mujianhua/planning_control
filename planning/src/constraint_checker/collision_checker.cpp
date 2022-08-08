#include "constraint_checker/collision_checker.h"
#include <cstddef>
#include <ros/ros.h>
#include "common/obstacle.h"
#include "common/vehicle_param.h"
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

CollisionChecker::CollisionChecker(
    const std::vector<const Obstacle *> &obstacles,
    const ReferenceLine *reference_line, const PlanningConfig *config)
    : config_(config), road_barrier_(*reference_line->GetRoadBarrier()) {
    for (auto obstacle : obstacles) {
        if (obstacle->IsStatic()) {
            static_obstacles_.push_back(*obstacle->PerceptionPolygon());
        } else {
            dynamic_obstacles_.emplace_back(
                *obstacle->PerceptionDynamicPolygons());
        }
    }
}

bool CollisionChecker::InCollision(double time, const math::Pose &pose,
                                   double collision_buffer) {

    math::AABox2d initial_box({-config_->vehicle.radius - collision_buffer,
                               -config_->vehicle.radius - collision_buffer},
                              {config_->vehicle.radius + collision_buffer,
                               config_->vehicle.radius + collision_buffer});

    double xr, yr, xf, yf;
    std::tie(xr, yr, xf, yf) =
        config_->vehicle.GetDiscPositions(pose.x(), pose.y(), pose.theta());

    auto f_box = initial_box, r_box = initial_box;
    f_box.Shift({xf, yf});
    r_box.Shift({xr, yr});

    if (CheckStaticCollision(math::Box2d(f_box)) ||
        CheckStaticCollision(math::Box2d(r_box)) ||
        CheckDynamicCollision(time, math::Box2d(f_box)) ||
        CheckDynamicCollision(time, math::Box2d(r_box))) {
        return true;
    }

    return false;
}

bool CollisionChecker::CheckCollision(double time, const math::Box2d &rect) {
    if (CheckDynamicCollision(time, rect)) {
        return true;
    }

    return CheckStaticCollision(rect);
}

bool CollisionChecker::CheckStaticCollision(const math::Box2d &rect) {

    for (auto &obstacle : static_obstacles_) {
        if (obstacle.HasOverlap(rect)) {
            return true;
        }
    }

    if (road_barrier_.empty()) {
        return false;
    }

    if (rect.max_x() < road_barrier_.front().x() ||
        rect.min_x() > road_barrier_.back().x()) {
        return false;
    }

    auto comp = [](double val, const math::Vec2d &a) { return val < a.x(); };

    // binary search
    auto check_start = std::upper_bound(
        road_barrier_.begin(), road_barrier_.end(), rect.min_x(), comp);
    auto check_end = std::upper_bound(road_barrier_.begin(),
                                      road_barrier_.end(), rect.max_x(), comp);

    if (check_start > road_barrier_.begin()) {
        std::advance(check_start, -1);
    }

    for (auto iter = check_start; iter != check_end; iter++) {
        if (rect.IsPointIn(*iter)) {
            return true;
        }
    }

    return false;
}

bool CollisionChecker::CheckDynamicCollision(double time,
                                             const math::Box2d &rect) {

    for (auto &obstacle : dynamic_obstacles_) {
        if (obstacle.front().first > time || obstacle.back().first < time) {
            continue;
        }
        auto result = std::upper_bound(
            obstacle.begin(), obstacle.end(), time,
            [](double val, const std::pair<double, math::Polygon2d> &ob) {
                return val < ob.first;
            });

        if (result->second.HasOverlap(rect)) {
            return true;
        }
    }

    return false;
}

} // namespace planning
} // namespace mujianhua
