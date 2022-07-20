#include "data_struct/reference_path_impl.h"

namespace mujianhua {
namespace planning {

ReferencePathImpl::ReferencePathImpl()
    : x_s_(new tk::spline), y_s_(new tk::spline), original_x_s_(new tk::spline),
      original_y_s_(new tk::spline) {}

ReferencePathImpl::~ReferencePathImpl() {
    delete x_s_;
    delete y_s_;
    delete original_x_s_;
    delete original_y_s_;
}

void ReferencePathImpl::clear() { reference_points_.clear(); }

void ReferencePathImpl::SetSpline(const tk::spline &x_s, const tk::spline &y_s,
                                  double max_s) {
    *x_s_ = x_s;
    *y_s_ = y_s;
    max_s_ = max_s;
}

const tk::spline &ReferencePathImpl::GetXS() const { return *x_s_; }
const tk::spline &ReferencePathImpl::GetYS() const { return *y_s_; }

double ReferencePathImpl::GetLength() const { return max_s_; }

} // namespace planning
} // namespace mujianhua