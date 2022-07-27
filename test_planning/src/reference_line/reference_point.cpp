#include "reference_line/reference_point.h"

namespace mujianhua {
namespace planning {

ReferencePoint::ReferencePoint(double x, double y, double s, double theta,
                               double kappa)
    : x_(x), y_(y), s_(s), theta_(theta), kappa_(kappa) {}

const double &ReferencePoint::x() const { return x_; }

const double &ReferencePoint::y() const { return y_; }

const double &ReferencePoint::s() const { return s_; }

const double &ReferencePoint::theta() const { return theta_; }

const double &ReferencePoint::kappa() const { return kappa_; }

double &ReferencePoint::x() { return x_; }

double &ReferencePoint::y() { return y_; }

double &ReferencePoint::s() { return s_; }

double &ReferencePoint::theta() { return theta_; }

double &ReferencePoint::kappa() { return kappa_; }

} // namespace planning
} // namespace mujianhua
