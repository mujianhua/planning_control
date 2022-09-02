#include "lattice/end_condition_sampler.h"

namespace planning {

EndConditionSampler::EndConditionSampler(
    const std::array<double, 3> &init_s, const std::array<double, 3> &init_d,
    std::shared_ptr<PathTimeGraph> path_time_graph)
    : init_s_(init_s),
      init_d_(init_d),
      path_time_graph_(std::move(path_time_graph)) {}

}  // namespace planning
