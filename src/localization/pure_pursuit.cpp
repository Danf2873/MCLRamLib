#include "localization/pure_pursuit.hpp"
#include <algorithm>
#include <cmath>

namespace localization {

PurePursuitController::PurePursuitController(double lookaheadDistance,
                                             double maxLinearVelocity)
    : lookaheadDistance(lookaheadDistance),
      maxLinearVelocity(maxLinearVelocity) {}

void PurePursuitController::setLookaheadDistance(double distance) {
  lookaheadDistance = distance;
}

void PurePursuitController::setMaxLinearVelocity(double velocity) {
  maxLinearVelocity = velocity;
}

std::pair<ChassisSpeeds, int> PurePursuitController::computeControl(
    const Pose2D &current, const Trajectory &trajectory, int lastIndex) const {

  const auto &points = trajectory.getPoints();
  if (points.empty())
    return {{0, 0}, 0};

  // 1. Find lookahead point
  Pose2D target = points.back().pose;
  int nextIndex = lastIndex;

  for (int i = lastIndex; i < (int)points.size() - 1; ++i) {
    double t = findIntersection(points[i].pose, points[i + 1].pose, current,
                                lookaheadDistance);
    if (t != -1) {
      target.x =
          points[i].pose.x + t * (points[i + 1].pose.x - points[i].pose.x);
      target.y =
          points[i].pose.y + t * (points[i + 1].pose.y - points[i].pose.y);
      nextIndex = i;
      break;
    }
  }

  // 2. Calculate curvature
  // Transform target to robot local coordinates
  double dx = target.x - current.x;
  double dy = target.y - current.y;

  // Rotate by -current.theta
  double localX = dx * std::cos(-current.theta) - dy * std::sin(-current.theta);
  double localY = dx * std::sin(-current.theta) + dy * std::cos(-current.theta);

  // Curvature gamma = 2 * y / L^2
  double L2 = localX * localX + localY * localY;
  double curvature = (L2 > 1e-6) ? (2.0 * localY / L2) : 0;

  // 3. Determine velocities
  // Simple constant velocity for now, could be scaled by curvature
  double v = maxLinearVelocity;

  // Limit velocity near end of path
  double distToEnd = current.distanceTo(points.back().pose);
  if (distToEnd < lookaheadDistance) {
    v *= (distToEnd / lookaheadDistance);
  }
  if (v < 2.0 && distToEnd > 0.5)
    v = 2.0; // Minimum crawl speed

  double w = v * curvature;

  return {{v, w}, nextIndex};
}

// Geometric helper for circle-line intersection
double PurePursuitController::findIntersection(const Pose2D &start,
                                               const Pose2D &end,
                                               const Pose2D &center,
                                               double radius) {
  double dx = end.x - start.x;
  double dy = end.y - start.y;
  double fx = start.x - center.x;
  double fy = start.y - center.y;

  double a = dx * dx + dy * dy;
  double b = 2 * (fx * dx + fy * dy);
  double c = fx * fx + fy * fy - radius * radius;

  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    return -1; // No intersection
  }

  discriminant = std::sqrt(discriminant);
  double t1 = (-b - discriminant) / (2 * a);
  double t2 = (-b + discriminant) / (2 * a);

  if (t1 >= 0 && t1 <= 1)
    return t1;
  if (t2 >= 0 && t2 <= 1)
    return t2;

  return -1;
}

} // namespace localization
