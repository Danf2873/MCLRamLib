#ifndef LOCALIZATION_MCL_CHASSIS_HPP
#define LOCALIZATION_MCL_CHASSIS_HPP

#include "localization/chassis_controller.hpp"
#include "localization/localization_manager.hpp"
#include "localization/trajectory.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <vector>

namespace localization {

/**
 * @brief EZ-Template-style all-in-one chassis with MCL localization.
 *
 * One constructor, one object, dead-simple autonomous commands.
 * Manages its own update loop, sensors, and localization stack.
 *
 * Example:
 * @code
 *   MCLChassis chassis({1, 2}, {-3, -4}, 5, 4.0, 12.5);
 *   chassis.add_sensor(6, {5, 0, 0});
 *   chassis.set_field_walls();
 *   chassis.set_pose(24, 24, 0);
 *
 *   // Autonomous:
 *   chassis.drive_to(48, 48);
 *   chassis.turn_to(90);
 *   chassis.pose_to(72, 72, 180);
 * @endcode
 */
class MCLChassis {
public:
  // ─── Construction ──────────────────────────────────────────

  /**
   * @brief Create a fully-configured chassis in one call.
   * @param left_ports  Left motor ports (negative = reversed).
   * @param right_ports Right motor ports (negative = reversed).
   * @param imu_port    Inertial sensor port.
   * @param wheel_diameter Wheel diameter in inches (e.g. 4.0 for 4" wheels).
   * @param track_width Distance between wheel centers in inches.
   * @param gear_ratio  Motor-to-wheel gear ratio (1.0 for direct drive).
   * @param rpm         Motor cartridge RPM: 600 (blue), 200 (green), 100 (red).
   */
  MCLChassis(std::initializer_list<std::int8_t> left_ports,
             std::initializer_list<std::int8_t> right_ports, int imu_port,
             double wheel_diameter, double track_width, double gear_ratio = 1.0,
             int rpm = 600);

  ~MCLChassis();

  // ─── Sensor Registration ──────────────────────────────────

  /**
   * @brief Add a distance sensor.
   * @param port Sensor port number.
   * @param forward_offset Inches forward from robot center (negative = back).
   * @param lateral_offset Inches left from center (negative = right).
   * @param facing_angle Angle sensor faces in degrees (0=front, 90=left,
   * 180=back, -90=right).
   */
  void add_sensor(int port, double forward_offset = 0,
                  double lateral_offset = 0, double facing_angle = 0);

  // ─── Field Map ────────────────────────────────────────────

  /**
   * @brief Sets the standard VEX field walls (12ft × 12ft = 144" × 144").
   */
  void set_field_walls();

  /**
   * @brief Adds a custom wall segment to the field map.
   * @param x1,y1 Start point in inches.
   * @param x2,y2 End point in inches.
   */
  void add_wall(double x1, double y1, double x2, double y2);

  // ─── Pose Control ─────────────────────────────────────────

  /**
   * @brief Set the robot's current position (call at start of auto).
   * @param x X position in inches.
   * @param y Y position in inches.
   * @param heading Heading in degrees (0 = right, 90 = up).
   */
  void set_pose(double x, double y, double heading);

  /**
   * @brief Get the robot's current X position (inches).
   */
  double get_x() const;

  /**
   * @brief Get the robot's current Y position (inches).
   */
  double get_y() const;

  /**
   * @brief Get the robot's current heading (degrees).
   */
  double get_heading() const;

  /**
   * @brief Get the current MCL confidence (0-100%).
   */
  double get_confidence() const;

  // ─── Movement Commands ────────────────────────────────────

  /**
   * @brief Drive to a point on the field.
   * @param x Target X (inches).
   * @param y Target Y (inches).
   * @param timeout Max time in ms (default 5000).
   */
  void drive_to(double x, double y, int timeout = 5000);

  /**
   * @brief Drive to a specific pose (position + heading).
   * @param x Target X (inches).
   * @param y Target Y (inches).
   * @param heading Target heading in degrees.
   * @param timeout Max time in ms (default 5000).
   */
  void pose_to(double x, double y, double heading, int timeout = 5000);

  /**
   * @brief Turn in place to a heading.
   * @param heading Target heading in degrees.
   * @param timeout Max time in ms (default 3000).
   */
  void turn_to(double heading, int timeout = 3000);

  /**
   * @brief Follow a trajectory.
   * @param trajectory The trajectory to follow.
   * @param timeout Max time in ms (default 10000).
   */
  void follow(const Trajectory &trajectory, int timeout = 10000);

  /**
   * @brief Follow a path using the Pure Pursuit controller.
   * @param poses Vector of poses defining the path.
   * @param lookahead Lookahead distance in inches (default 12.0).
   * @param timeout Max time in ms (default 10000).
   */
  void pure_pursuit(const std::vector<Pose2D> &poses, double lookahead = 12.0,
                    int timeout = 10000);

  /**
   * @brief Stop all motors (brake).
   */
  void stop();

private:
  // Hardware
  std::unique_ptr<pros::MotorGroup> left_motors;
  std::unique_ptr<pros::MotorGroup> right_motors;
  std::unique_ptr<pros::Imu> imu;
  std::vector<std::unique_ptr<pros::Distance>> dist_sensors;

  // Localization stack
  std::shared_ptr<DifferentialOdometry> odometry;
  std::shared_ptr<MCLLocalizer> mcl;
  std::shared_ptr<LocalizationManager> manager;
  std::unique_ptr<ChassisController> controller;

  // Field
  FieldMap field;

  // Config
  RobotGeometry geometry;

  // Background task
  std::unique_ptr<pros::Task> update_task;
  bool running = true;

  void start_update_loop();
  static double deg_to_rad(double deg);
  static double rad_to_deg(double rad);
};

} // namespace localization

#endif // LOCALIZATION_MCL_CHASSIS_HPP
