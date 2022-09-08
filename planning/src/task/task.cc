#include "task/task.h"

namespace planning {

Task::Task(const std::string& name) : name_(name) {}

const std::string& Task::Name() const { return name_; }

bool Task::Init(const PlanningConfig& config) { return true; }

bool Task::Execute(Frame* frame) {
  frame_ = frame;
  return true;
}

}  // namespace planning
