#pragma once

#include <memory>

#include "lattice/path_time_graph.h"

namespace planning {

class EndConditionSampler {
 public:
  EndConditionSampler(const std::array<double, 3> &init_s,
                      const std::array<double, 3> &init_d,
                      std::shared_ptr<PathTimeGraph> path_time_graph);

 private:
  std::array<double, 3> init_s_;
  std::array<double, 3> init_d_;
  std::shared_ptr<PathTimeGraph> path_time_graph_;
};

}  // namespace planning
