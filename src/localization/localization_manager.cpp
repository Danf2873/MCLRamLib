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

void LocalizationManager::sensorUpdate(double measuredDistance,
                                       const FieldMap &map) {
  // 1. MCL Weighting step
  mcl->updateFromDistanceSensor(measuredDistance, map,
                                geometry.distanceSensorOffset);

  // 2. Resampling
  mcl->resample();
}

Pose2D LocalizationManager::getOdometryPose() const {
  return odometry->getPose();
}

Pose2D LocalizationManager::getFusedPose() const { return mcl->estimatePose(); }

void LocalizationManager::resetPose(const Pose2D &pose) {
  odometry->reset(pose);
  mcl->setPose(pose);
  lastOdometryPose = pose;
}

} // namespace localization
