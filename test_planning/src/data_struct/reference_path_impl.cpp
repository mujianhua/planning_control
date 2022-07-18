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

} // namespace planning
} // namespace mujianhua