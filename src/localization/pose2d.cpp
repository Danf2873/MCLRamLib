#include "localization/pose2d.hpp"

namespace localization {

double Pose2D::normalizeAngle(double angle) {
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}

double Pose2D::distanceTo(const Pose2D &other) const {
  return std::hypot(other.x - x, other.y - y);
}

double Pose2D::angleTo(const Pose2D &other) const {
  return std::atan2(other.y - y, other.x - x);
}

Pose2D Pose2D::operator+(const Pose2D &other) const {
  return {x + other.x, y + other.y, normalizeAngle(theta + other.theta)};
}

Pose2D Pose2D::operator-(const Pose2D &other) const {
  return {x - other.x, y - other.y, normalizeAngle(theta - other.theta)};
}

} // namespace localization
