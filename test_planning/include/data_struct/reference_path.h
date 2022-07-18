/**
 * @file
 * @brief
 */

#include <memory>
#include "data_struct/reference_path_impl.h"

#pragma once

namespace mujianhua {
namespace planning {

class ReferencePath {
  public:
    ReferencePath();

  private:
    std::shared_ptr<ReferencePathImpl> reference_path_impl;
};

} // namespace planning
} // namespace mujianhua