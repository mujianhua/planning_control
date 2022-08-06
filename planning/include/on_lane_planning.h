#pragma once

#include <memory>
#include "planning_base.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_line_provider.h"

namespace mujianhua {
namespace planning {

class OnLanePlanning : public PlanningBase {
  public:
    explicit OnLanePlanning(
        const std::shared_ptr<common::DependencyInjector> &injector)
        : PlanningBase(injector) {}

    bool Init() override;

    void RunOnce(const LocalView &local_view) override;

    void UpdateReferenceLine(const ReferenceLine *reference_line) {
        reference_line_ = reference_line;
    }

  private:
    bool InitFrame(const uint32_t sequence_num);

  private:
    std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
    const ReferenceLine *reference_line_{};
};

} // namespace planning
} // namespace mujianhua
