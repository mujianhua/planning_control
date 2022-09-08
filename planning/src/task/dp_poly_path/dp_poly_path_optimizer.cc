#include "task/dp_poly_path/dp_poly_path_optimizer.h"

namespace planning {

DpPolyPathOptimizer::DpPolyPathOptimizer()
    : PathOptimizer("DpPolyPathOptimizer") {}

bool DpPolyPathOptimizer::Init(const PlanningConfig& config) {
  is_init_ = true;
}

bool DpPolyPathOptimizer::Process(const ReferenceLine& reference_line,
                                  const TrajectoryPoint& init_point) {
  return false;
}

}  // namespace planning
