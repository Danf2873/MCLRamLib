#include "localization/mcl.hpp"
#include <algorithm>
#include <cmath>

namespace localization {

double FieldMap::expectedDistance(const Pose2D &pose) const {
  double minT = 1000.0; // Max sensing range
  double cosTheta = std::cos(pose.theta);
  double sinTheta = std::sin(pose.theta);

  for (const auto &wall : walls) {
    double dx = wall.x2 - wall.x1;
    double dy = wall.y2 - wall.y1;

    double det = -cosTheta * dy + sinTheta * dx;
    if (std::abs(det) < 1e-6)
      continue;

    double dt = -(wall.x1 - pose.x) * dy + (wall.y1 - pose.y) * dx;
    double du = cosTheta * (wall.y1 - pose.y) - sinTheta * (wall.x1 - pose.x);

    double t = dt / det;
    double u = du / det;

    if (t > 0 && u >= 0 && u <= 1) {
      if (t < minT)
        minT = t;
    }
  }
  return minT;
}

MCLLocalizer::MCLLocalizer(int numParticles) : numParticles(numParticles) {
  particles.resize(numParticles);
  std::random_device rd;
  gen.seed(rd());
}

void MCLLocalizer::initialize(double minX, double maxX, double minY,
                              double maxY) {
  std::uniform_real_distribution<double> distX(minX, maxX);
  std::uniform_real_distribution<double> distY(minY, maxY);
  std::uniform_real_distribution<double> distTheta(-M_PI, M_PI);

  for (auto &p : particles) {
    p.pose = {distX(gen), distY(gen), distTheta(gen)};
    p.weight = 1.0 / numParticles;
  }
}

void MCLLocalizer::predict(double v, double w, double dt) {
  std::normal_distribution<double> noiseV(0, alpha1 * std::abs(v) +
                                                 alpha2 * std::abs(w));
  std::normal_distribution<double> noiseW(0, alpha3 * std::abs(v) +
                                                 alpha4 * std::abs(w));

  for (auto &p : particles) {
    double noisyV = v + noiseV(gen);
    double noisyW = w + noiseW(gen);

    if (std::abs(noisyW) < 1e-6) {
      p.pose.x += noisyV * dt * std::cos(p.pose.theta);
      p.pose.y += noisyV * dt * std::sin(p.pose.theta);
    } else {
      double radius = noisyV / noisyW;
      p.pose.x += radius * (std::sin(p.pose.theta + noisyW * dt) -
                            std::sin(p.pose.theta));
      p.pose.y += radius * (std::cos(p.pose.theta) -
                            std::cos(p.pose.theta + noisyW * dt));
      p.pose.theta = Pose2D::normalizeAngle(p.pose.theta + noisyW * dt);
    }
  }
}

void MCLLocalizer::updateFromDistanceSensor(double measuredDistance,
                                            const FieldMap &map,
                                            const Pose2D &sensorOffset) {
  double totalWeight = 0;
  for (auto &p : particles) {
    // Compute sensor pose for this particle
    double sCos = std::cos(p.pose.theta);
    double sSin = std::sin(p.pose.theta);
    Pose2D sensorPose;
    sensorPose.x = p.pose.x + sensorOffset.x * sCos - sensorOffset.y * sSin;
    sensorPose.y = p.pose.y + sensorOffset.x * sSin + sensorOffset.y * sCos;
    sensorPose.theta =
        Pose2D::normalizeAngle(p.pose.theta + sensorOffset.theta);

    double expected = map.expectedDistance(sensorPose);

    // Gaussian likelihood
    double error = measuredDistance - expected;
    double likelihood =
        std::exp(-(error * error) / (2 * sigmaDistance * sigmaDistance));

    // Multiply current weight (Bayesian update)
    p.weight *= likelihood;
    totalWeight += p.weight;
  }

  if (totalWeight > 1e-9) {
    for (auto &p : particles)
      p.weight /= totalWeight;
  } else {
    // If all weights zero, reset weights uniformly
    for (auto &p : particles)
      p.weight = 1.0 / numParticles;
  }
}

void MCLLocalizer::resample() {
  std::vector<Particle> newParticles;
  newParticles.reserve(numParticles);

  std::uniform_real_distribution<double> dist(0, 1.0 / numParticles);
  double r = dist(gen);
  double c = particles[0].weight;
  int i = 0;

  for (int m = 0; m < numParticles; ++m) {
    double u = r + (double)m / numParticles;
    while (u > c && i < numParticles - 1) {
      i++;
      c += particles[i].weight;
    }
    newParticles.push_back(particles[i]);
    newParticles.back().weight = 1.0 / numParticles;
  }
  particles = std::move(newParticles);
}

Pose2D MCLLocalizer::estimatePose() const {
  double meanX = 0, meanY = 0, meanCos = 0, meanSin = 0;
  for (const auto &p : particles) {
    meanX += p.pose.x * p.weight;
    meanY += p.pose.y * p.weight;
    meanCos += std::cos(p.pose.theta) * p.weight;
    meanSin += std::sin(p.pose.theta) * p.weight;
  }
  return {meanX, meanY, std::atan2(meanSin, meanCos)};
}

double MCLLocalizer::getConfidence() const {
  // Compute weighted variance of particle positions
  Pose2D mean = estimatePose();
  double varX = 0, varY = 0;
  for (const auto &p : particles) {
    double dx = p.pose.x - mean.x;
    double dy = p.pose.y - mean.y;
    varX += dx * dx * p.weight;
    varY += dy * dy * p.weight;
  }
  double spread = std::sqrt(varX + varY);

  // Map spread to confidence: spread=0 -> 1.0, spread>=10" -> ~0.0
  // Using exponential decay: confidence = exp(-spread / scale)
  double scale = 3.0; // inches — tune this for your field
  return std::exp(-spread / scale);
}

void MCLLocalizer::setPose(const Pose2D &pose, double stdev) {
  std::normal_distribution<double> distX(pose.x, stdev);
  std::normal_distribution<double> distY(pose.y, stdev);
  std::normal_distribution<double> distTheta(pose.theta, stdev * 0.1);

  for (auto &p : particles) {
    p.pose = {distX(gen), distY(gen), distTheta(gen)};
    p.weight = 1.0 / numParticles;
  }
}

} // namespace localization
