#ifndef LOCALIZATION_ROBOT_PORTS_HPP
#define LOCALIZATION_ROBOT_PORTS_HPP

#include <cstdint>

namespace localization {

/**
 * @brief Centralized port assignments for the robot.
 */
struct RobotPorts {
  // Drive motors
  int8_t leftDrivePort;
  int8_t rightDrivePort;

  // Sensors
  int8_t imuPort;
  int8_t distanceSensorPort;

  // Optional Tracking Wheels
  int8_t leftTrackPort;
  int8_t rightTrackPort;
  int8_t middleTrackPort;
};

extern const RobotPorts DEFAULT_PORTS;

} // namespace localization

#endif // LOCALIZATION_ROBOT_PORTS_HPP
