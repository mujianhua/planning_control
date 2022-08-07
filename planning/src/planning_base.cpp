#include "planning_base.h"

namespace mujianhua {
namespace planning {

bool PlanningBase::Init(const PlanningConfig &config) {
    config_ = config;
    return true;
}

} // namespace planning
} // namespace mujianhua
