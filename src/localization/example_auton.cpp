#include "localization/chassis_controller.hpp"
#include "localization/localization_manager.hpp"
#include "localization/robot_ports.hpp"
#include "localization/trajectory.hpp"
#include "main.h"
#include <memory>

using namespace localization;

/**
 * @brief Full example: linked sensors, Bayes filter, and concise movement.
 */
void example_autonomous() {
  // ── 1. Hardware ──────────────────────────────────────────────
  pros::MotorGroup left_motors({(std::int8_t)DEFAULT_PORTS.leftDrivePort},
                               pros::v5::MotorGears::blue);
  pros::MotorGroup right_motors({(std::int8_t)DEFAULT_PORTS.rightDrivePort},
                                pros::v5::MotorGears::blue);
  pros::Imu imu(DEFAULT_PORTS.imuPort);

  // Distance sensors
  pros::Distance frontSensor(DEFAULT_PORTS.distanceSensorPort);
  pros::Distance leftSensor(8);  // Example: port 8
  pros::Distance rightSensor(9); // Example: port 9

  // ── 2. Localization Stack ────────────────────────────────────
  auto odometry = std::make_shared<DifferentialOdometry>(
      &left_motors, &right_motors, &imu, DEFAULT_GEOMETRY);
  auto mcl = std::make_shared<MCLLocalizer>(200);
  auto manager =
      std::make_shared<LocalizationManager>(odometry, mcl, DEFAULT_GEOMETRY);

  // ── 3. Register Sensors with Offsets (once!) ─────────────────
  //  offset = {x_forward, y_left, facing_angle} relative to robot center
  manager->addSensor({&frontSensor, {5.0, 0.0, 0.0}});        // front
  manager->addSensor({&leftSensor, {0.0, 5.0, M_PI / 2}});    // left side
  manager->addSensor({&rightSensor, {0.0, -5.0, -M_PI / 2}}); // right side

  // ── 4. Chassis Controller ───────────────────────────────────
  ChassisController chassis(&left_motors, &right_motors, manager.get(),
                            DEFAULT_GEOMETRY);

  // ── 5. Field Map (VEX field boundary walls) ─────────────────
  FieldMap field = {
      {{0, 0, 144, 0}, {144, 0, 144, 144}, {144, 144, 0, 144}, {0, 144, 0, 0}}};

  // ── 6. Starting Position ────────────────────────────────────
  manager->resetPose({24.0, 24.0, 0.0});

  // ── 7. Background Update Task ───────────────────────────────
  // Runs odometry + MCL sensor fusion at 100Hz
  pros::Task updateTask([&]() {
    while (true) {
      manager->update(0.01);            // Odometry + MCL predict
      manager->updateAllSensors(field); // Read all sensors, fuse, resample

      // Telemetry: print Bayes-fused pose and confidence
      Pose2D pose = manager->getFusedPose();
      double conf = manager->getMCLConfidence();
      pros::lcd::print(0, "X:%.1f Y:%.1f T:%.1f", pose.x, pose.y, pose.theta);
      pros::lcd::print(1, "MCL Conf: %.0f%%", conf * 100.0);

      pros::delay(10);
    }
  });

  // ── 8. Autonomous Routine (short & concise) ─────────────────
  chassis.moveToPoint(48.0, 48.0);            // Drive to (48, 48)
  chassis.moveToPose({72.0, 72.0, M_PI / 2}); // Drive to pose
  chassis.moveToPoint(24.0, 24.0);            // Return home

  // Trajectory following example
  std::vector<TrajectoryPoint> points = {{0.0, {24, 24, 0}, 10.0, 0.0},
                                         {1.0, {48, 24, 0}, 20.0, 0.0},
                                         {2.0, {72, 48, M_PI / 2}, 10.0, 0.0},
                                         {3.0, {72, 72, M_PI / 2}, 0.0, 0.0}};
  Trajectory traj(points);
  chassis.follow(traj);

  chassis.stop();
}
