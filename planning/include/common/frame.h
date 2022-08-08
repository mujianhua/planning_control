#pragma once

#include "common/local_view.h"
#include "common/obstacle.h"
#include "config/planning_config.h"
#include "constraint_checker/collision_checker.h"
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

class Frame {
  public:
    Frame(uint32_t sequence_num, const ReferenceLine *reference_line,
          const LocalView &local_view, const PlanningConfig &config);

    const ReferenceLine *reference_line() const { return reference_line_; }

    const std::vector<const Obstacle *> obstacles() const;

    uint32_t SequenceNum() const;

    bool InCollision(double time, const math::Pose &pose,
                     double collision_buffer = 0.0) const;

    bool CheckCollision(double time, const math::Box2d &rect) const;

  private:
    bool CheckStaticCollision(const math::Box2d &rect) const;

    bool CheckDynamicCollision(double time, const math::Box2d &rect) const;

    uint32_t sequence_num_ = 0;
    const ReferenceLine *reference_line_;
    IndexedObstacles obstacles_;
    LocalView local_view_;

    PlanningConfig config_;

    std::vector<DynamicObstacle> dynamic_obstacles_;
    std::vector<math::Polygon2d> static_obstacles_;
    std::vector<math::Vec2d> road_barrier_;
};

} // namespace planning
} // namespace mujianhua
