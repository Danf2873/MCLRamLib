#include "localization/robot_geometry.hpp"
#include "localization/robot_ports.hpp"

namespace localization {

// Example configuration - User should edit these values
const RobotGeometry DEFAULT_GEOMETRY = {
    // Drivetrain
    2.0,   // wheelRadius (4" wheels)
    12.5,  // trackWidth
    1.0,   // gearRatio
    300.0, // encoderTicksPerRev (600 RPM cartridges)

    // Tracking wheels
    false, // hasTrackingWheels
    1.375, // trackWheelRadius (2.75" wheels)
    10.0,  // trackWheelTrackWidth
    0.0,   // trackWheelCenterOffset

    // Distance sensor
    {5.0, 0.0, 0.0} // distanceSensorOffset (5" forward from center)
};

const RobotPorts DEFAULT_PORTS = {
    1, // leftDrivePort
    2, // rightDrivePort
    3, // imuPort
    4, // distanceSensorPort
    0, // leftTrackPort (0 = unused)
    0, // rightTrackPort
    0  // middleTrackPort
};

} // namespace localization
