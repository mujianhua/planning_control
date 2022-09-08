#pragma once

#include "task/path_optimizer.h"

namespace planning {

class DpPolyPathOptimizer : public PathOptimizer {
 public:
  DpPolyPathOptimizer();

  bool Init(const PlanningConfig& config) override;

 private:
  /**
   * \brief main dp poly path optimizer interface.
   **/
  bool Process(const ReferenceLine& reference_line,
               const TrajectoryPoint& init_point) override;
};

}  // namespace planning
