#pragma once

#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

class ReferenceLineProvider {
  public:
    ReferenceLineProvider() = default;

    bool GetReferenceLine(ReferenceLine *reference_line);

    /**
     * TODO: 应该从 HD map 自己更新,后期需要更改
     */
    void UpdateReferenceLine(const ReferenceLine &reference_line);

  private:
    ReferenceLine reference_line_;
};

} // namespace planning
} // namespace mujianhua
