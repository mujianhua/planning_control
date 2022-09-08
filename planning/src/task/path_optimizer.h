
#pragma once

#include "common/data_struct.h"
#include "reference_line/reference_line.h"
#include "task/task.h"

namespace planning {

class PathOptimizer : public Task {
 public:
  explicit PathOptimizer(const std::string& name);

  virtual ~PathOptimizer() = default;

  /**
   * \brief main running interface.
   **/
  bool Execute(Frame* frame) override;

 protected:
  virtual bool Process(const ReferenceLine& reference_line,
                       const TrajectoryPoint& init_point) = 0;
};

}  // namespace planning
