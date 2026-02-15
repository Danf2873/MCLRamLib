#ifndef LOCALIZATION_PURE_PURSUIT_HPP
#define LOCALIZATION_PURE_PURSUIT_HPP

#include "localization/pose2d.hpp"
#include "localization/ramsete.hpp" // For ChassisSpeeds
#include "localization/trajectory.hpp"
#include <vector>


namespace localization {

/**
 * @brief Pure Pursuit path-following controller.
 *
 * Geometric controller that finds a "lookahead point" on a path and calculates
 * the curvature needed to reach it.
 */
class PurePursuitController {
public:
  /**
   * @brief Constructor.
   * @param lookaheadDistance Distance ahead of the robot to find target points
   * (inches).
   * @param maxLinearVelocity Maximum linear velocity (inches/sec).
   */
  PurePursuitController(double lookaheadDistance = 12.0,
                        double maxLinearVelocity = 20.0);

  /**
   * @brief Computes the next chassis speeds to follow the trajectory.
   * @param current Current pose of the robot.
   * @param trajectory The trajectory being followed.
   * @param lastIndex The index of the path point found in the previous
   * iteration.
   * @return Pair of {Commanded speeds, new lastIndex}.
   */
  std::pair<ChassisSpeeds, int> computeControl(const Pose2D &current,
                                               const Trajectory &trajectory,
                                               int lastIndex) const;

  /**
   * @brief Sets the lookahead distance.
   * @param distance Lookahead distance in inches.
   */
  void setLookaheadDistance(double distance);

  /**
   * @brief Sets the maximum linear velocity.
   * @param velocity Max velocity in inches/sec.
   */
  void setMaxLinearVelocity(double velocity);

private:
  double lookaheadDistance;
  double maxLinearVelocity;

  /**
   * @brief Find the intersection between a circle (lookahead) and a line
   * segment (path).
   */
  static double findIntersection(const Pose2D &start, const Pose2D &end,
                                 const Pose2D &center, double radius);
};

} // namespace localization

#endif // LOCALIZATION_PURE_PURSUIT_HPP
