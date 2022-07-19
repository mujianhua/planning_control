#include "data_struct/reference_path.h"

namespace mujianhua {
namespace planning {

ReferencePath::ReferencePath()
    : reference_path_impl(std::make_shared<ReferencePathImpl>()) {}

void ReferencePath::clear() { return reference_path_impl->clear(); }

} // namespace planning
} // namespace mujianhua
