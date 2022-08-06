#include "reference_line/reference_line_provider.h"

namespace mujianhua {
namespace planning {

bool ReferenceLineProvider::GetReferenceLine(ReferenceLine *reference_line) {
    if (!reference_line) {
        return false;
    }
    reference_line = &reference_line_;
    return true;
}

void ReferenceLineProvider::UpdateReferenceLine(
    const ReferenceLine &reference_line) {
    reference_line_ = reference_line;
}

} // namespace planning
} // namespace mujianhua