#include "localization/chassis_controller.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace localization {

ChassisController::ChassisController(pros::MotorGroup *leftMotors,
                                     pros::MotorGroup *rightMotors,
                                     LocalizationManager *manager,
                                     const RobotGeometry &geometry)
    : leftMotors(leftMotors), rightMotors(rightMotors), manager(manager),
      geometry(geometry), ramsete(2.0, 0.7) {}

void ChassisController::applySpeeds(ChassisSpeeds speeds) {
  // Convert speeds (v, w) to wheel velocities
  double vl = speeds.v - (speeds.w * geometry.trackWidth / 2.0);
  double vr = speeds.v + (speeds.w * geometry.trackWidth / 2.0);

  // Convert inches/sec to RPM
  double leftRPM = (vl / (2 * M_PI * geometry.wheelRadius)) * 60.0;
  double rightRPM = (vr / (2 * M_PI * geometry.wheelRadius)) * 60.0;

  leftMotors->move_velocity(leftRPM);
  rightMotors->move_velocity(rightRPM);
}

void ChassisController::moveToPose(Pose2D target, double timeoutMs) {
  // For a simple moveToPose, we can generate a 1-second linear path or use a
  // PID. Here, we'll use a simple constant-velocity move for demonstration of
  // the "short" API. In a production lib, this might use a S-Curve or
  // Mini-Trajectory.
  std::vector<TrajectoryPoint> points = {
      {0.0, manager->getFusedPose(), 0.0, 0.0},
      {timeoutMs / 1000.0, target, 0.0, 0.0}};
  follow(Trajectory(points), timeoutMs);
}

void ChassisController::moveToPoint(double x, double y, double timeoutMs) {
  Pose2D current = manager->getFusedPose();
  double angle = std::atan2(y - current.y, x - current.x);
  moveToPose({x, y, angle}, timeoutMs);
}

void ChassisController::follow(const Trajectory &trajectory, double timeoutMs) {
  uint32_t startTime = pros::millis();
  double dt = 0.01;

  while (pros::millis() - startTime < timeoutMs) {
    double currentTime = (pros::millis() - startTime) / 1000.0;
    if (currentTime > trajectory.getDuration() + 0.2)
      break; // Allow small buffer

    manager->update(dt);
    Pose2D currentPose = manager->getFusedPose();

    TrajectoryPoint target = trajectory.sample(currentTime);
    ChassisSpeeds speeds =
        ramsete.computeControl(currentPose, target.pose, target.v, target.w);

    applySpeeds(speeds);
    pros::delay(dt * 1000);
  }
  stop();
}

void ChassisController::stop() {
  leftMotors->brake();
  rightMotors->brake();
}

} // namespace localization
