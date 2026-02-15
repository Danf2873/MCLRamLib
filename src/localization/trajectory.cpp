#include "localization/trajectory.hpp"
#include <algorithm>

namespace localization {

TrajectoryPoint Trajectory::sample(double t) const {
  if (points.empty())
    return {t, {0, 0, 0}, 0, 0};
  if (t <= points.front().time)
    return points.front();
  if (t >= points.back().time)
    return points.back();

  auto it = std::lower_bound(
      points.begin(), points.end(), t,
      [](const TrajectoryPoint &p, double time) { return p.time < time; });

  const TrajectoryPoint &end = *it;
  const TrajectoryPoint &start = *(it - 1);

  double dt = end.time - start.time;
  if (dt == 0)
    return start;

  double alpha = (t - start.time) / dt;

  TrajectoryPoint result;
  result.time = t;
  result.pose.x = start.pose.x + alpha * (end.pose.x - start.pose.x);
  result.pose.y = start.pose.y + alpha * (end.pose.y - start.pose.y);
  result.pose.theta =
      start.pose.theta +
      alpha * Pose2D::normalizeAngle(end.pose.theta - start.pose.theta);
  result.v = start.v + alpha * (end.v - start.v);
  result.w = start.w + alpha * (end.w - start.w);

  return result;
}

double Trajectory::getDuration() const {
  if (points.empty())
    return 0;
  return points.back().time;
}

} // namespace localization
