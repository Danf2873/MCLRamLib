#include "localization/ramsete.hpp"
#include <cmath>

namespace localization {

RamseteController::RamseteController(double b, double zeta)
    : b(b), zeta(zeta) {}

ChassisSpeeds RamseteController::computeControl(const Pose2D &current,
                                                const Pose2D &desired,
                                                double v_ref,
                                                double w_ref) const {
  double dx = desired.x - current.x;
  double dy = desired.y - current.y;
  double dtheta = Pose2D::normalizeAngle(desired.theta - current.theta);

  // Transform error to robot frame
  double ex = dx * std::cos(current.theta) + dy * std::sin(current.theta);
  double ey = -dx * std::sin(current.theta) + dy * std::cos(current.theta);
  double etheta = dtheta;

  // k gain calculation
  double k = 2.0 * zeta * std::sqrt(w_ref * w_ref + b * v_ref * v_ref);

  // Sinc function approximation for small angles
  double sinc_theta =
      (std::abs(etheta) < 1e-6) ? 1.0 : std::sin(etheta) / etheta;

  double v = v_ref * std::cos(etheta) + k * ex;
  double w = w_ref + b * v_ref * sinc_theta * ey + k * etheta;

  return {v, w};
}

} // namespace localization
