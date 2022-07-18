#include "data_struct/reference_path.h"

namespace mujianhua {
namespace planning {

ReferencePath::ReferencePath()
    : reference_path_impl(std::make_shared<ReferencePathImpl>()) {}

} // namespace planning
} // namespace mujianhua
