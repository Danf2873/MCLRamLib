#include "localization/mcl_chassis.hpp"
#include <cmath>

namespace localization {

// ─── Helpers ──────────────────────────────────────────────────

double MCLChassis::deg_to_rad(double deg) { return deg * M_PI / 180.0; }
double MCLChassis::rad_to_deg(double rad) { return rad * 180.0 / M_PI; }

// ─── Constructor ──────────────────────────────────────────────

MCLChassis::MCLChassis(std::initializer_list<std::int8_t> left_ports,
                       std::initializer_list<std::int8_t> right_ports,
                       int imu_port, double wheel_diameter, double track_width,
                       double gear_ratio, int rpm) {
  // Build geometry from simple user inputs
  geometry = {};
  geometry.wheelRadius = wheel_diameter / 2.0;
  geometry.trackWidth = track_width;
  geometry.gearRatio = gear_ratio;
  geometry.hasTrackingWheels = false;

  // Calculate ticks from RPM
  switch (rpm) {
  case 100:
    geometry.encoderTicksPerRev = 1800.0;
    break;
  case 200:
    geometry.encoderTicksPerRev = 900.0;
    break;
  case 600:
  default:
    geometry.encoderTicksPerRev = 300.0;
    break;
  }

  // Create hardware
  left_motors = std::make_unique<pros::MotorGroup>(left_ports);
  right_motors = std::make_unique<pros::MotorGroup>(right_ports);
  imu = std::make_unique<pros::Imu>(imu_port);

  // Create localization stack
  odometry = std::make_shared<DifferentialOdometry>(
      left_motors.get(), right_motors.get(), imu.get(), geometry);
  mcl = std::make_shared<MCLLocalizer>(200);
  manager = std::make_shared<LocalizationManager>(odometry, mcl, geometry);
  controller = std::make_unique<ChassisController>(
      left_motors.get(), right_motors.get(), manager.get(), geometry);

  // Default field walls
  set_field_walls();

  // Start background update
  start_update_loop();
}

MCLChassis::~MCLChassis() {
  running = false;
  if (update_task) {
    update_task->remove();
  }
}

// ─── Background Loop ─────────────────────────────────────────

void MCLChassis::start_update_loop() {
  update_task = std::make_unique<pros::Task>([this]() {
    while (running) {
      manager->update(0.01);
      manager->updateAllSensors(field);
      pros::delay(10);
    }
  });
}

// ─── Sensors ─────────────────────────────────────────────────

void MCLChassis::add_sensor(int port, double forward_offset,
                            double lateral_offset, double facing_angle) {
  dist_sensors.push_back(std::make_unique<pros::Distance>(port));
  manager->addSensor(
      {dist_sensors.back().get(),
       {forward_offset, lateral_offset, deg_to_rad(facing_angle)}});
}

// ─── Field Map ───────────────────────────────────────────────

void MCLChassis::set_field_walls() {
  field.walls.clear();
  field.walls.push_back({0, 0, 144, 0});     // bottom
  field.walls.push_back({144, 0, 144, 144}); // right
  field.walls.push_back({144, 144, 0, 144}); // top
  field.walls.push_back({0, 144, 0, 0});     // left
}

void MCLChassis::add_wall(double x1, double y1, double x2, double y2) {
  field.walls.push_back({x1, y1, x2, y2});
}

// ─── Pose ────────────────────────────────────────────────────

void MCLChassis::set_pose(double x, double y, double heading) {
  manager->resetPose({x, y, deg_to_rad(heading)});
}

double MCLChassis::get_x() const { return manager->getFusedPose().x; }

double MCLChassis::get_y() const { return manager->getFusedPose().y; }

double MCLChassis::get_heading() const {
  return rad_to_deg(manager->getFusedPose().theta);
}

double MCLChassis::get_confidence() const {
  return manager->getMCLConfidence() * 100.0;
}

// ─── Movement ────────────────────────────────────────────────

void MCLChassis::drive_to(double x, double y, int timeout) {
  controller->moveToPoint(x, y, timeout);
}

void MCLChassis::pose_to(double x, double y, double heading, int timeout) {
  controller->moveToPose({x, y, deg_to_rad(heading)}, timeout);
}

void MCLChassis::turn_to(double heading, int timeout) {
  // Turn in place: move to current position with new heading
  Pose2D current = manager->getFusedPose();
  controller->moveToPose({current.x, current.y, deg_to_rad(heading)}, timeout);
}

void MCLChassis::follow(const Trajectory &trajectory, int timeout) {
  controller->follow(trajectory, timeout);
}

void MCLChassis::stop() { controller->stop(); }

} // namespace localization
