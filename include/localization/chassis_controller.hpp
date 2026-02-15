#ifndef LOCALIZATION_CHASSIS_CONTROLLER_HPP
#define LOCALIZATION_CHASSIS_CONTROLLER_HPP

#include "localization/localization_manager.hpp"
#include "localization/ramsete.hpp"
#include "localization/trajectory.hpp"
#include "pros/motor_group.hpp"

namespace localization {

/**
 * @brief High-level movement commands for the robot.
 */
class ChassisController {
public:
  /**
   * @brief Constructor.
   * @param leftMotors Pointer to left motor group.
   * @param rightMotors Pointer to right motor group.
   * @param manager Pointer to localization manager.
   * @param geometry Robot geometry configuration.
   */
  ChassisController(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors,
                    LocalizationManager *manager,
                    const RobotGeometry &geometry);

  /**
   * @brief Moves directly to a target pose (x, y, theta).
   * @param target The target Pose2D.
   * @param timeoutMs Maximum time to allow for the move.
   */
  void moveToPose(Pose2D target, double timeoutMs = 2000);

  /**
   * @brief Moves to a specific point (x, y) ignoring final heading.
   * @param x Target X coordinate.
   * @param y Target Y coordinate.
   * @param timeoutMs Maximum time to allow for the move.
   */
  void moveToPoint(double x, double y, double timeoutMs = 2000);

  /**
   * @brief Follows a trajectory using the RAMSETE controller.
   * @param trajectory The trajectory to follow.
   * @param timeoutMs Maximum time to allow for the trajectory.
   */
  void follow(const Trajectory &trajectory, double timeoutMs = 5000);

  /**
   * @brief Stops the chassis.
   */
  void stop();

private:
  pros::MotorGroup *leftMotors;
  pros::MotorGroup *rightMotors;
  LocalizationManager *manager;
  RobotGeometry geometry;
  RamseteController ramsete;

  void applySpeeds(ChassisSpeeds speeds);
};

} // namespace localization

#endif // LOCALIZATION_CHASSIS_CONTROLLER_HPP
