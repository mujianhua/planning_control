#include "common/math/vec2d.h"
#include <cmath>
#include <complex>
#include "glog/logging.h"

namespace mujianhua {
namespace planning {
namespace math {

Vec2d Vec2d::CreatUnitVec2d(const double angle) {
    return Vec2d{std::cos(angle), std::sin(angle)};
}

double Vec2d::Length() const { return std::hypot(x_, y_); }

double Vec2d::LengthSquare() const { return x_ * x_ + y_ * y_; }

double Vec2d::Angle() const { return std::atan2(y_, x_); }

void Vec2d::Normalize() {
    const double l = Length();
    if (l > kMathEpsilon) {
        x_ /= l;
        y_ /= l;
    }
}

double Vec2d::DistanceTo(const Vec2d &other) const {
    return std::hypot(x_ - other.x(), y_ - other.y());
}

double Vec2d::DistanceSquareTo(const Vec2d &other) const {
    const double dx = x_ - other.x();
    const double dy = y_ - other.y();
    return dx * dx + dy * dy;
}

// TODO:
Vec2d Vec2d::rotate(const double angle) const {
    return Vec2d{x_ * cos(angle) - y_ * sin(angle),
                 x_ * sin(angle) + y_ * cos(angle)};
}

void Vec2d::SelfRotate(const double angle) {}

Vec2d Vec2d::operator+(const Vec2d &other) const {
    return Vec2d{x_ + other.x(), y_ + other.y()};
}

Vec2d Vec2d::operator-(const Vec2d &other) const {
    return Vec2d{x_ - other.x(), y_ - other.y()};
}

Vec2d Vec2d::operator*(const double ratio) const {
    return Vec2d{x_ * ratio, y_ * ratio};
}

Vec2d Vec2d::operator/(const double ratio) const {
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    return Vec2d{x_ / ratio, y_ / ratio};
}

Vec2d &Vec2d::operator+=(const Vec2d &other) {
    x_ += other.x();
    y_ += other.y();
    return *this;
}

Vec2d &Vec2d::operator-=(const Vec2d &other) {
    x_ -= other.x();
    y_ -= other.y();
    return *this;
}

Vec2d &Vec2d::operator*=(const double ratio) {
    x_ *= ratio;
    y_ *= ratio;
    return *this;
}

Vec2d &Vec2d::operator/=(const double ratio) {
    x_ /= ratio;
    y_ /= ratio;
    return *this;
}

bool Vec2d::operator==(const Vec2d &other) const {
    return (std::abs(x_ - other.x()) < kMathEpsilon &&
            std::abs(y_ - other.y()) < kMathEpsilon);
}

std::string Vec2d::DebugString() const { return ""; }

} // namespace math
} // namespace planning
} // namespace mujianhua