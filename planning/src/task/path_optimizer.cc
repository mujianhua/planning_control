#include "task/path_optimizer.h"

namespace planning {

PathOptimizer::PathOptimizer(const std::string& name) : Task(name) {}

bool PathOptimizer::Execute(Frame* frame) {
  Task::Execute(frame);
  TrajectoryPoint init_point;
  auto ret = Process(frame_->reference_line(), init_point);
}

}  // namespace planning
