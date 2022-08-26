
#include "frame.h"

#include "../reference_line/reference_line.h"
#include "../visualization/plot.h"

namespace planning {

constexpr double kSampleStep = 0.1;

void Frame::Update(const LocalView &local_view) {
  // vehicle state
  vehicle_state_ = *local_view.vehicle_state;

  // obstacles
  // TODO:
  obstacles_ = local_view.obstacles->Items();
  for (const auto &obs : local_view.obstacles->Items()) {
    if (obs->IsStatic()) {
      static_obstacles_.push_back(obs);
    } else {
      dynamic_obstacles_.push_back(obs);
    }
  }
}

void Frame::SetReferenceLine(const ReferenceLine &reference) {
  reference_line_ = reference;

  road_barrier_.clear();

  double start_s = reference_line_.reference_points().front().s;
  double back_s = reference_line_.reference_points().back().s;
  int sample_points = int((back_s - start_s) / kSampleStep);
  for (int i = 0; i <= sample_points; i++) {
    double s = start_s + i * kSampleStep;
    auto ref = reference_line_.GetMatchPoint(s);

    road_barrier_.push_back(reference_line_.GetCartesian(s, ref.left_bound));
    road_barrier_.push_back(reference_line_.GetCartesian(s, -ref.right_bound));
  }
  std::sort(road_barrier_.begin(), road_barrier_.end(),
            [](const Vec2d &a, const Vec2d &b) { return a.x() < b.x(); });
}

bool Frame::CheckStaticCollision(const math::Box2d &rect) {
  for (auto &obstacle : static_obstacles_) {
    if (obstacle->polygon2d().HasOverlap(rect)) {
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

  auto comp = [](double val, const Vec2d &a) { return val < a.x(); };

  // binary search
  auto check_start = std::upper_bound(road_barrier_.begin(),
                                      road_barrier_.end(), rect.min_x(), comp);
  auto check_end = std::upper_bound(road_barrier_.begin(), road_barrier_.end(),
                                    rect.max_x(), comp);

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

bool Frame::CheckCollision(double time, const math::Box2d &rect) {
  if (CheckDynamicCollision(time, rect)) {
    return true;
  }

  return CheckStaticCollision(rect);
}

bool Frame::CheckOptimizationCollision(double time, const math::Pose &pose,
                                       double collision_buffer) {
  math::AABox2d initial_box({-config_.vehicle.radius - collision_buffer,
                             -config_.vehicle.radius - collision_buffer},
                            {config_.vehicle.radius + collision_buffer,
                             config_.vehicle.radius + collision_buffer});

  double xr, yr, xf, yf;
  std::tie(xr, yr, xf, yf) =
      config_.vehicle.GetDiscPositions(pose.x(), pose.y(), pose.theta());

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

bool Frame::CheckDynamicCollision(double time, const math::Box2d &rect) {
  for (auto &obstacle : dynamic_obstacles_) {
    if (obstacle->polygon2ds().front().first > time ||
        obstacle->polygon2ds().back().first < time) {
      continue;
    }
    auto result = std::upper_bound(
        obstacle->polygon2ds().begin(), obstacle->polygon2ds().end(), time,
        [](double val, const std::pair<double, math::Polygon2d> &ob) {
          return val < ob.first;
        });

    if (result->second.HasOverlap(rect)) {
      return true;
    }
  }

  return false;
}

std::unordered_map<int, math::Polygon2d> Frame::QueryDynamicObstacles(
    double time) {
  std::unordered_map<int, math::Polygon2d> filtered;
  int idx = 0;
  for (auto &obstacle : dynamic_obstacles_) {
    idx++;
    if (obstacle->polygon2ds().front().first > time ||
        obstacle->polygon2ds().back().first < time) {
      continue;
    }
    auto result = std::upper_bound(
        obstacle->polygon2ds().begin(), obstacle->polygon2ds().end(), time,
        [](double val, const std::pair<double, math::Polygon2d> &ob) {
          return val < ob.first;
        });

    filtered.insert({idx, result->second});
  }
  return filtered;
}

void Frame::Visualize() {
  std::vector<double> lb_x, lb_y, ub_x, ub_y;

  for (auto &point : reference_line_.reference_points()) {
    auto lb = reference_line_.GetCartesian(point.s, point.left_bound);
    auto ub = reference_line_.GetCartesian(point.s, -point.right_bound);

    lb_x.push_back(lb.x());
    lb_y.push_back(lb.y());
    ub_x.push_back(ub.x());
    ub_y.push_back(ub.y());
  }

  visualization::Plot(lb_x, lb_y, 0.1, visualization::Color::Grey, 1,
                      "Road Left");
  visualization::Plot(ub_x, ub_y, 0.1, visualization::Color::Grey, 1,
                      "Road Right");

  int idx = 0;
  for (auto &obstacle : static_obstacles_) {
    visualization::PlotPolygon(obstacle->polygon2d(), 0.1,
                               visualization::Color::Magenta, idx++,
                               "Obstacles");
  }

  // plot first frame of dynamic obstacles
  idx = 1;
  for (auto &obstacle : dynamic_obstacles_) {
    auto color = visualization::Color::fromHSV(
        int((double)idx / static_cast<double>(dynamic_obstacles_.size() * 320)),
        1.0, 1.0);
    color.set_alpha(0.5);
    visualization::PlotPolygon(obstacle->polygon2ds().at(0).second, 0.1, color,
                               idx, "Online Obstacle");
    idx++;
  }

  visualization::Trigger();
}

}  // namespace planning
