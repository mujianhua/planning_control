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

    void clear();

    void SetSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);

    const tk::spline &GetXS() const;
    const tk::spline &GetYS() const;

    double GetLength() const;

  private:
    std::shared_ptr<ReferencePathImpl> reference_path_impl_;
};

} // namespace planning
} // namespace mujianhua