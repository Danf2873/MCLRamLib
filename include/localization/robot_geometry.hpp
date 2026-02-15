#ifndef LOCALIZATION_ROBOT_GEOMETRY_HPP
#define LOCALIZATION_ROBOT_GEOMETRY_HPP

#include "localization/pose2d.hpp"

namespace localization {

/**
 * @brief Robot geometry constants.
 */
struct RobotGeometry {
  // Drivetrain geometry
  double wheelRadius; ///< Radius of drive wheels in inches
  double trackWidth;  ///< Distance between left and right wheels in inches
  double gearRatio;   ///< Gear ratio (motor rotations / wheel rotations)
  double
      encoderTicksPerRev; ///< Ticks per motor revolution (standard V5 is 1800
                          ///< for 100rpm, 900 for 200rpm, 300 for 600rpm)

  // Tracking wheel geometry (optional)
  bool hasTrackingWheels;
  double trackWheelRadius;
  double trackWheelTrackWidth;
  double trackWheelCenterOffset; ///< Offset of middle tracking wheel from
                                 ///< center (positive is forward)

  // Distance sensor mounting (relative to robot center)
  Pose2D distanceSensorOffset; ///< Offset and heading of the distance sensor
};

extern const RobotGeometry DEFAULT_GEOMETRY;

} // namespace localization

#endif // LOCALIZATION_ROBOT_GEOMETRY_HPP
