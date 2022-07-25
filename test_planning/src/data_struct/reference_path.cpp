
#include "data_struct/reference_path.h"

namespace mujianhua {
namespace planning {

ReferencePath::ReferencePath()
    : reference_path_impl_(std::make_shared<ReferencePathImpl>()) {}

void ReferencePath::clear() { return reference_path_impl_->clear(); }

void ReferencePath::SetSpline(const tk::spline &x_s, const tk::spline &y_s,
                              double max_s) {
    return reference_path_impl_->SetSpline(x_s, y_s, max_s);
}

const tk::spline &ReferencePath::GetXS() const {
    return reference_path_impl_->GetXS();
}

const tk::spline &ReferencePath::GetYS() const {
    return reference_path_impl_->GetYS();
}

double ReferencePath::GetXS(double s) const {
    return reference_path_impl_->GetXS()(s);
}
double ReferencePath::GetYS(double s) const {
    return reference_path_impl_->GetYS()(s);
}

double ReferencePath::GetLength() const {
    return reference_path_impl_->GetLength();
}

size_t ReferencePath::GetSize() const {
    return reference_path_impl_->GetSize();
}

const std::vector<TrajectoryPoint> &ReferencePath::GetReferencePoints() const {
    return reference_path_impl_->GetReferencePoints();
}

void ReferencePath::SetLength(double s) {
    return reference_path_impl_->SetLength(s);
}

bool ReferencePath::BuildReferenceFromSpline(double delta_s_smaller,
                                             double delta_s_larger) {
    return reference_path_impl_->BuildReferenceFromSpline(delta_s_smaller,
                                                          delta_s_larger);
}

bool ReferencePath::BuildReferenceFromStates(
    const std::vector<TrajectoryPoint> &states) {
    return reference_path_impl_->BuildReferenceFromStates(states);
}

void ReferencePath::UpdateBounds(const Map &map) {
    return reference_path_impl_->UpdateBoundsImproved(map);
}

const std::vector<VehicleBound> &ReferencePath::GetBounds() const {
    return reference_path_impl_->GetBounds();
}

std::shared_ptr<VehicleBound> ReferencePath::IsBlocked() const {
    return reference_path_impl_->IsBlocked();
}

} // namespace planning
} // namespace mujianhua
