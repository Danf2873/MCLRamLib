#ifndef LOCALIZATION_MCL_HPP
#define LOCALIZATION_MCL_HPP

#include "localization/pose2d.hpp"
#include <random>
#include <vector>


namespace localization {

/**
 * @brief A line segment representing a wall or boundary.
 */
struct LineSegment {
  double x1, y1, x2, y2;
};

/**
 * @brief Represents the VEX field as a collection of line segments.
 */
struct FieldMap {
  std::vector<LineSegment> walls;

  /**
   * @brief Computes expected distance to the nearest wall from a pose along its
   * heading.
   * @param pose The pose to cast a ray from.
   * @return Expected distance in inches.
   */
  double expectedDistance(const Pose2D &pose) const;
};

/**
 * @brief A single particle in the filter.
 */
struct Particle {
  Pose2D pose;
  double weight;
};

/**
 * @brief Monte Carlo Localization (Particle Filter).
 */
class MCLLocalizer {
public:
  /**
   * @brief Constructor.
   * @param numParticles Number of particles to maintain.
   */
  MCLLocalizer(int numParticles = 200);

  /**
   * @brief Initializes particles randomly within a region.
   * @param minX Minimum X position.
   * @param maxX Maximum X position.
   * @param minY Minimum Y position.
   * @param maxY Maximum Y position.
   */
  void initialize(double minX, double maxX, double minY, double maxY);

  /**
   * @brief Motion model: predicts next state based on odometry motion.
   * @param v Measured linear velocity.
   * @param w Measured angular velocity.
   * @param dt Time delta.
   */
  void predict(double v, double w, double dt);

  /**
   * @brief Measurement model: updates weights based on distance sensor reading.
   * @param measuredDistance Distance reading from sensor.
   * @param map Field map.
   * @param sensorOffset Offset of sensor on the robot.
   */
  void updateFromDistanceSensor(double measuredDistance, const FieldMap &map,
                                const Pose2D &sensorOffset);

  /**
   * @brief Low-variance resampling step.
   */
  void resample();

  /**
   * @brief Estimates the current pose from particles.
   * @return Weighted average pose.
   */
  Pose2D estimatePose() const;

  /**
   * @brief Sets the pose of all particles (reset filter).
   * @param pose The new pose.
   * @param stdev Standard deviation of noise to add.
   */
  void setPose(const Pose2D &pose, double stdev = 2.0);

private:
  int numParticles;
  std::vector<Particle> particles;
  mutable std::mt19937 gen;

  // Noise parameters
  double alpha1 = 0.05;       // rot noise from rot
  double alpha2 = 0.05;       // rot noise from trans
  double alpha3 = 0.05;       // trans noise from trans
  double alpha4 = 0.05;       // trans noise from rot
  double sigmaDistance = 2.0; // measurement noise (inches)
};

} // namespace localization

#endif // LOCALIZATION_MCL_HPP
