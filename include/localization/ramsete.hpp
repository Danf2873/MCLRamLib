#ifndef LOCALIZATION_RAMSETE_HPP
#define LOCALIZATION_RAMSETE_HPP

#include "localization/pose2d.hpp"

namespace localization {

/**
 * @brief Output of the RAMSETE controller.
 */
struct ChassisSpeeds {
  double v; ///< Linear velocity in inches/second
  double w; ///< Angular velocity in radians/second
};

/**
 * @brief Nonlinear controller for following trajectories.
 */
class RamseteController {
public:
  /**
   * @brief Constructor.
   * @param b Convergence parameter (b > 0), typically 2.0.
   * @param zeta Damping parameter (0 < zeta < 1), typically 0.7.
   */
  RamseteController(double b = 2.0, double zeta = 0.7);

  /**
   * @brief Computes the next chassis speeds to follow the target.
   * @param current Current pose.
   * @param desired Desired pose from trajectory.
   * @param v_ref Reference linear velocity from trajectory.
   * @param w_ref Reference angular velocity from trajectory.
   * @return Commanded velocities.
   */
  ChassisSpeeds computeControl(const Pose2D &current, const Pose2D &desired,
                               double v_ref, double w_ref) const;

private:
  double b;
  double zeta;
};

} // namespace localization

#endif // LOCALIZATION_RAMSETE_HPP
