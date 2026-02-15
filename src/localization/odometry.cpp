#include "localization/odometry.hpp"
#include <cmath>

namespace localization {

DifferentialOdometry::DifferentialOdometry(pros::MotorGroup *leftMotors,
                                           pros::MotorGroup *rightMotors,
                                           pros::Imu *imu,
                                           const RobotGeometry &geometry)
    : leftMotors(leftMotors), rightMotors(rightMotors), imu(imu),
      geometry(geometry), pose({0, 0, 0}) {}

void DifferentialOdometry::setTrackingWheels(pros::Rotation *left,
                                             pros::Rotation *right,
                                             pros::Rotation *middle) {
  leftTrack = left;
  rightTrack = right;
  middleTrack = middle;
}

void DifferentialOdometry::update(double dt) {
  if (!initialized) {
    if (imu) {
      lastHeading = imu->get_heading() * M_PI / 180.0;
    }
    if (leftTrack && rightTrack) {
      lastLeftPos = leftTrack->get_position() / 100.0 * (M_PI / 180.0) *
                    geometry.trackWheelRadius;
      lastRightPos = rightTrack->get_position() / 100.0 * (M_PI / 180.0) *
                     geometry.trackWheelRadius;
      if (middleTrack) {
        lastMiddlePos = middleTrack->get_position() / 100.0 * (M_PI / 180.0) *
                        geometry.trackWheelRadius;
      }
    } else {
      // Use motor encoders
      std::vector<double> leftPositions = leftMotors->get_position_all();
      std::vector<double> rightPositions = rightMotors->get_position_all();
      double avgLeft = 0;
      for (double p : leftPositions)
        avgLeft += p;
      avgLeft /= leftPositions.size();
      double avgRight = 0;
      for (double p : rightPositions)
        avgRight += p;
      avgRight /= rightPositions.size();

      lastLeftPos = (avgLeft / geometry.encoderTicksPerRev) *
                    geometry.gearRatio * (2 * M_PI * geometry.wheelRadius);
      lastRightPos = (avgRight / geometry.encoderTicksPerRev) *
                     geometry.gearRatio * (2 * M_PI * geometry.wheelRadius);
    }
    initialized = true;
    return;
  }

  double currentLeft = 0, currentRight = 0, currentMiddle = 0,
         currentHeading = 0;

  // Get current heading
  if (imu) {
    currentHeading = imu->get_heading() * M_PI / 180.0;
  } else {
    // Fallback or estimated heading from differential drive if no IMU
  }

  // Get current positions in inches
  if (leftTrack && rightTrack) {
    currentLeft = (leftTrack->get_position() / 36000.0) *
                  (2 * M_PI * geometry.trackWheelRadius);
    currentRight = (rightTrack->get_position() / 36000.0) *
                   (2 * M_PI * geometry.trackWheelRadius);
    if (middleTrack) {
      currentMiddle = (middleTrack->get_position() / 36000.0) *
                      (2 * M_PI * geometry.trackWheelRadius);
    }
  } else {
    std::vector<double> leftPositions = leftMotors->get_position_all();
    std::vector<double> rightPositions = rightMotors->get_position_all();
    double avgLeft = 0;
    for (double p : leftPositions)
      avgLeft += p;
    avgLeft /= leftPositions.size();
    double avgRight = 0;
    for (double p : rightPositions)
      avgRight += p;
    avgRight /= rightPositions.size();

    currentLeft = (avgLeft / geometry.encoderTicksPerRev) * geometry.gearRatio *
                  (2 * M_PI * geometry.wheelRadius);
    currentRight = (avgRight / geometry.encoderTicksPerRev) *
                   geometry.gearRatio * (2 * M_PI * geometry.wheelRadius);
  }

  double dLeft = currentLeft - lastLeftPos;
  double dRight = currentRight - lastRightPos;
  double dHeading = Pose2D::normalizeAngle(currentHeading - lastHeading);

  double dCenter = (dLeft + dRight) / 2.0;

  // If we have a middle tracking wheel, we can use it for lateral movement if
  // the robot is omni-directional but for differential drive, it's mostly for
  // better accuracy during turns.
  double dMiddle = 0;
  if (middleTrack) {
    dMiddle = currentMiddle - lastMiddlePos;
  }

  // Update pose using Arc length or small-angle approximation
  // Here we use the small-angle approximation for simplicity, but could be
  // improved to arc-based.
  double avgHeading = lastHeading + dHeading / 2.0;

  double dx = dCenter * std::cos(avgHeading);
  double dy = dCenter * std::sin(avgHeading);

  // If middle wheel is perpendicular to the center:
  if (middleTrack) {
    // Correct for rotation of the middle wheel
    double sideOffset = dMiddle - (dHeading * geometry.trackWheelCenterOffset);
    dx -= sideOffset * std::sin(avgHeading);
    dy += sideOffset * std::cos(avgHeading);
  }

  pose.x += dx;
  pose.y += dy;
  pose.theta = currentHeading;

  lastLeftPos = currentLeft;
  lastRightPos = currentRight;
  lastMiddlePos = currentMiddle;
  lastHeading = currentHeading;
}

void DifferentialOdometry::reset(const Pose2D &newPose) {
  pose = newPose;
  initialized = false; // Re-initialize state on next update
}

} // namespace localization
