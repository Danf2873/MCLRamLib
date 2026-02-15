#include "localization/localization_manager.hpp"

namespace localization {

LocalizationManager::LocalizationManager(
    std::shared_ptr<DifferentialOdometry> odometry,
    std::shared_ptr<MCLLocalizer> mcl, const RobotGeometry &geometry)
    : odometry(odometry), mcl(mcl), geometry(geometry) {
  lastOdometryPose = odometry->getPose();
}

void LocalizationManager::update(double dt) {
  // 1. Update odometry
  odometry->update(dt);
  Pose2D currentOdometryPose = odometry->getPose();

  // 2. Compute delta motion for MCL prediction
  // We can use the simple v, w from odometry delta
  double dx = currentOdometryPose.x - lastOdometryPose.x;
  double dy = currentOdometryPose.y - lastOdometryPose.y;
  double dtheta = Pose2D::normalizeAngle(currentOdometryPose.theta -
                                         lastOdometryPose.theta);

  double dcenter = std::sqrt(dx * dx + dy * dy);
  // Determine sign of v based on theta
  double moveAngle = std::atan2(dy, dx);
  if (std::abs(Pose2D::normalizeAngle(moveAngle - currentOdometryPose.theta)) >
      M_PI / 2) {
    dcenter = -dcenter;
  }

  double v = dcenter / dt;
  double w = dtheta / dt;

  // 3. MCL Prediction step
  mcl->predict(v, w, dt);

  lastOdometryPose = currentOdometryPose;
}

void LocalizationManager::addSensor(const DistanceSensorConfig &config) {
  sensors.push_back(config);
}

void LocalizationManager::updateAllSensors(const FieldMap &map) {
  bool anyUpdated = false;

  for (const auto &cfg : sensors) {
    double distMM = cfg.sensor->get_distance();
    double distInches = distMM / 25.4;
    int confidence = cfg.sensor->get_confidence();

    // Skip readings that are out of range or low confidence
    if (distInches < cfg.minRange || distInches > cfg.maxRange)
      continue;
    if (confidence < cfg.minConfidence)
      continue;

    // Update MCL weights (no resample yet — accumulate across all sensors)
    mcl->updateFromDistanceSensor(distInches, map, cfg.offset);
    anyUpdated = true;
  }

  // Single resample after all sensor updates
  if (anyUpdated) {
    mcl->resample();
  }
}

void LocalizationManager::sensorUpdate(double measuredDistance,
                                       const FieldMap &map,
                                       std::optional<Pose2D> offset,
                                       bool resampleStep) {
  // 1. MCL Weighting step
  mcl->updateFromDistanceSensor(measuredDistance, map,
                                offset.value_or(geometry.distanceSensorOffset));

  // 2. Resampling (if requested)
  if (resampleStep) {
    mcl->resample();
  }
}

void LocalizationManager::resample() { mcl->resample(); }

Pose2D LocalizationManager::getOdometryPose() const {
  return odometry->getPose();
}

Pose2D LocalizationManager::getFusedPose() const {
  Pose2D odomPose = odometry->getPose();
  Pose2D mclPose = mcl->estimatePose();
  double mclConf = mcl->getConfidence();

  // Bayesian weighted fusion:
  // High confidence (close to 1.0) → trust MCL
  // Low confidence (close to 0.0) → trust odometry
  double w = mclConf;

  Pose2D fused;
  fused.x = w * mclPose.x + (1.0 - w) * odomPose.x;
  fused.y = w * mclPose.y + (1.0 - w) * odomPose.y;

  // Circular mean for heading
  double cosTheta =
      w * std::cos(mclPose.theta) + (1.0 - w) * std::cos(odomPose.theta);
  double sinTheta =
      w * std::sin(mclPose.theta) + (1.0 - w) * std::sin(odomPose.theta);
  fused.theta = std::atan2(sinTheta, cosTheta);

  return fused;
}

Pose2D LocalizationManager::getMCLPose() const { return mcl->estimatePose(); }

double LocalizationManager::getMCLConfidence() const {
  return mcl->getConfidence();
}

void LocalizationManager::resetPose(const Pose2D &pose) {
  odometry->reset(pose);
  mcl->setPose(pose);
  lastOdometryPose = pose;
}

} // namespace localization
