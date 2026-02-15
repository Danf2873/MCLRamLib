#ifndef LOCALIZATION_TRAJECTORY_HPP
#define LOCALIZATION_TRAJECTORY_HPP

#include "localization/pose2d.hpp"
#include <vector>

namespace localization {

/**
 * @brief A single point in a trajectory.
 */
struct TrajectoryPoint {
  double time; ///< Time since start of trajectory in seconds
  Pose2D pose; ///< Target pose
  double v;    ///< Target linear velocity in inches/second
  double w;    ///< Target angular velocity in radians/second
};

/**
 * @brief Represents a path as a sequence of timestamped points.
 */
class Trajectory {
public:
  Trajectory() = default;
  Trajectory(const std::vector<TrajectoryPoint> &points) : points(points) {}

  /**
   * @brief Creates a trajectory from a list of positions (geometrically
   * spaced).
   * @param poses List of poses (x, y, theta).
   * @param velocity Average velocity to assume for timing (inches/sec).
   */
  static Trajectory fromPoses(const std::vector<Pose2D> &poses,
                              double velocity = 20.0);

  /**
   * @brief Samples the trajectory at a given time.
   * @param t Time in seconds.
   * @return Interpolated TrajectoryPoint.
   */
  TrajectoryPoint sample(double t) const;

  /**
   * @brief Gets the total duration of the trajectory.
   * @return Duration in seconds.
   */
  double getDuration() const;

  /**
   * @brief Gets the underlying trajectory points.
   */
  const std::vector<TrajectoryPoint> &getPoints() const { return points; }

private:
  std::vector<TrajectoryPoint> points;
};

} // namespace localization

#endif // LOCALIZATION_TRAJECTORY_HPP
