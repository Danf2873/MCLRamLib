#ifndef LOCALIZATION_ODOMETRY_HPP
#define LOCALIZATION_ODOMETRY_HPP

#include "localization/pose2d.hpp"
#include "localization/robot_geometry.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <memory>
#include <vector>

namespace localization {

/**
 * @brief Handles odometry calculations for a differential drive robot.
 */
class DifferentialOdometry {
public:
  /**
   * @brief Constructor for motor-encoder-based odometry.
   * @param leftMotors Pointer to left motor group.
   * @param rightMotors Pointer to right motor group.
   * @param imu Pointer to inertial sensor.
   * @param geometry Robot geometry configuration.
   */
  DifferentialOdometry(pros::MotorGroup *leftMotors,
                       pros::MotorGroup *rightMotors, pros::Imu *imu,
                       const RobotGeometry &geometry);

  /**
   * @brief Sets optional tracking wheels.
   * @param left Pointer to left tracking wheel encoder.
   * @param right Pointer to right tracking wheel encoder.
   * @param middle Pointer to middle/back tracking wheel encoder.
   */
  void setTrackingWheels(pros::Rotation *left, pros::Rotation *right,
                         pros::Rotation *middle);

  /**
   * @brief Updates the robot's pose.
   * @param dt Time delta since last update in seconds.
   */
  void update(double dt);

  /**
   * @brief Gets the current pose.
   * @return The current Pose2D.
   */
  Pose2D getPose() const { return pose; }

  /**
   * @brief Resets the odometry to a specific pose.
   * @param newPose The pose to reset to.
   */
  void reset(const Pose2D &newPose);

private:
  pros::MotorGroup *leftMotors;
  pros::MotorGroup *rightMotors;
  pros::Imu *imu;

  pros::Rotation *leftTrack = nullptr;
  pros::Rotation *rightTrack = nullptr;
  pros::Rotation *middleTrack = nullptr;

  RobotGeometry geometry;
  Pose2D pose;

  double lastLeftPos = 0;
  double lastRightPos = 0;
  double lastMiddlePos = 0;
  double lastHeading = 0;

  bool initialized = false;
};

} // namespace localization

#endif // LOCALIZATION_ODOMETRY_HPP
