#ifndef LOCALIZATION_MANAGER_HPP
#define LOCALIZATION_MANAGER_HPP

#include "localization/mcl.hpp"
#include "localization/odometry.hpp"
#include "localization/robot_geometry.hpp"
#include "pros/distance.hpp"
#include <optional>
#include <vector>

namespace localization {

/**
 * @brief A distance sensor paired with its mounting offset.
 */
struct DistanceSensorConfig {
  pros::Distance *sensor; ///< Pointer to the PROS distance sensor.
  Pose2D offset; ///< Mounting position & angle relative to robot center.
  double minRange = 1.0;  ///< Minimum valid reading (inches).
  double maxRange = 80.0; ///< Maximum valid reading (inches).
  int minConfidence = 50; ///< Minimum confidence threshold (0-100).
};

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
   * @brief Registers a distance sensor with its offset.
   * @param config The sensor configuration.
   */
  void addSensor(const DistanceSensorConfig &config);

  /**
   * @brief Reads all registered sensors and updates MCL weights, then
   * resamples.
   * @param map The field map.
   */
  void updateAllSensors(const FieldMap &map);

  /**
   * @brief Updates MCL with a single manual distance reading.
   * @param measuredDistance Distance in inches.
   * @param map The field map.
   * @param offset Optional sensor offset. If not provided, uses geometry
   * default.
   * @param resample Whether to perform resampling after this update.
   */
  void sensorUpdate(double measuredDistance, const FieldMap &map,
                    std::optional<Pose2D> offset = std::nullopt,
                    bool resample = true);

  /**
   * @brief Manually triggers MCL resampling.
   */
  void resample();

  /**
   * @brief Gets the pure odometry pose.
   */
  Pose2D getOdometryPose() const;

  /**
   * @brief Gets the MCL-fused pose (Bayesian blend of odom + MCL).
   */
  Pose2D getFusedPose() const;

  /**
   * @brief Gets the raw MCL pose (particle filter estimate only).
   */
  Pose2D getMCLPose() const;

  /**
   * @brief Gets the MCL confidence (0.0 - 1.0). Useful for telemetry.
   */
  double getMCLConfidence() const;

  /**
   * @brief Resets all localization to a specific pose.
   */
  void resetPose(const Pose2D &pose);

private:
  std::shared_ptr<DifferentialOdometry> odometry;
  std::shared_ptr<MCLLocalizer> mcl;
  RobotGeometry geometry;

  Pose2D lastOdometryPose;
  std::vector<DistanceSensorConfig> sensors;
};

} // namespace localization

#endif // LOCALIZATION_MANAGER_HPP
