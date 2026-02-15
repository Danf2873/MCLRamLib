#ifndef LOCALIZATION_POSE2D_HPP
#define LOCALIZATION_POSE2D_HPP

#include <cmath>

namespace localization {

/**
 * @brief Represents a 2D pose (x, y, theta) in field coordinates.
 */
struct Pose2D {
    double x;     ///< X position in inches
    double y;     ///< Y position in inches
    double theta; ///< Heading in radians

    /**
     * @brief Normalizes an angle to the range [-PI, PI].
     * @param angle The angle to normalize.
     * @return The normalized angle.
     */
    static double normalizeAngle(double angle);

    /**
     * @brief Computes the distance to another pose.
     * @param other The other pose.
     * @return The Euclidean distance.
     */
    double distanceTo(const Pose2D& other) const;

    /**
     * @brief Computes the angle to another pose.
     * @param other The other pose.
     * @return The angle in radians.
     */
    double angleTo(const Pose2D& other) const;

    /**
     * @brief Pose addition (composition).
     * @param other The pose to add.
     * @return The resulting pose.
     */
    Pose2D operator+(const Pose2D& other) const;

    /**
     * @brief Pose subtraction.
     * @param other The pose to subtract.
     * @return The resulting pose.
     */
    Pose2D operator-(const Pose2D& other) const;
};

} // namespace localization

#endif // LOCALIZATION_POSE2D_HPP
