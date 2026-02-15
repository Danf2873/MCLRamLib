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

Trajectory Trajectory::fromPoses(const std::vector<Pose2D> &poses,
                                 double velocity) {
  std::vector<TrajectoryPoint> p;
  double currentTime = 0;
  Pose2D lastPose = {0, 0, 0};

  for (size_t i = 0; i < poses.size(); ++i) {
    if (i > 0) {
      double dist = poses[i].distanceTo(lastPose);
      currentTime += dist / velocity;
    }
    // We don't have w (angular velocity) easily,
    // but Pure Pursuit doesn't strictly need it.
    p.push_back({currentTime, poses[i], velocity, 0.0});
    lastPose = poses[i];
  }
  return Trajectory(p);
}

} // namespace localization
