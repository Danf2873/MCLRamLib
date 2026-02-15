#ifndef LOCALIZATION_MANAGER_HPP
#define LOCALIZATION_MANAGER_HPP

#include "localization/mcl.hpp"
#include "localization/odometry.hpp"
#include "localization/robot_geometry.hpp"


namespace localization {

/**
 * @brief High-level manager for all localization components.
 */
class LocalizationManager {
public:
  /**
   * @brief Constructor.
   * @param odometry Shared pointer to odometry instance.
   * @param mcl Shared pointer to MCL instance.
   * @param geometry Robot geometry.
   */
  LocalizationManager(std::shared_ptr<DifferentialOdometry> odometry,
                      std::shared_ptr<MCLLocalizer> mcl,
                      const RobotGeometry &geometry);

  /**
   * @brief Main update loop to be called periodically.
   * @param dt Time delta in seconds.
   */
  void update(double dt);

  /**
   * @brief Updates MCL with a distance sensor reading.
   * @param measuredDistance Distance in inches.
   * @param map The field map.
   */
  void sensorUpdate(double measuredDistance, const FieldMap &map);

  /**
   * @brief Gets the pure odometry pose.
   */
  Pose2D getOdometryPose() const;

  /**
   * @brief Gets the MCL-fused pose.
   */
  Pose2D getFusedPose() const;

  /**
   * @brief Resets all localization to a specific pose.
   */
  void resetPose(const Pose2D &pose);

private:
  std::shared_ptr<DifferentialOdometry> odometry;
  std::shared_ptr<MCLLocalizer> mcl;
  RobotGeometry geometry;

  Pose2D lastOdometryPose;
};

} // namespace localization

#endif // LOCALIZATION_MANAGER_HPP
