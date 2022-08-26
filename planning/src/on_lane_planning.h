#pragma once

#include "planning/planning_config.h"
#include "planning_base.h"
#include "reference_line/reference_line_provider.h"

namespace planning {

class OnLanePlanning : public PlanningBase {
 public:
  explicit OnLanePlanning(const PlanningConfig &config);

  ~OnLanePlanning();

  void RunOnce(const LocalView &local_view,
               DiscretizedTrajectory *const adc_trajectory) override;

 private:
  void UpdateFrame();

 private:
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
};

}  // namespace planning
