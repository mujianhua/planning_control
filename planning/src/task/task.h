#pragma once

#include <string>

#include "common/planning_config.h"
#include "planning/frame.h"

namespace planning {

class Task {
 public:
  explicit Task(const std::string& name);

  virtual ~Task() = default;

  virtual const std::string& Name() const;

  virtual bool Init(const PlanningConfig& config);

  /**
   * \brief main interface.
   **/
  virtual bool Execute(Frame* frame);

 protected:
  bool is_init_ = false;
  Frame* frame_ = nullptr;

 private:
  const std::string name_;
};

}  // namespace planning
