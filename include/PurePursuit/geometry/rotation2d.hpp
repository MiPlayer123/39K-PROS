#pragma once

#include "okapi/api/units/QAngle.hpp"

using namespace okapi::literals;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle
 * (cosine and sine).
 */
class Rotation2d {
public:
    /**
     * Constructs a Rotation2d with a default angle of 0 degrees.
     */
    constexpr Rotation2d() = default;
  
    /**
     * Constructs a Rotation2d with the given radian value.
     *
     * @param value The value of the angle in radians.
     */
    Rotation2d(okapi::QAngle value);  // NOLINT(runtime/explicit)
  
    /**
     * Constructs a Rotation2d with the given x and y (cosine and sine)
     * components. The x and y don't have to be normalized.
     *
     * @param x The x component or cosine of the rotation.
     * @param y The y component or sine of the rotation.
     */
    Rotation2d(double x, double y);
  
    /**
     * Adds two rotations together, with the result being bounded between -pi and
     * pi.
     *
     * For example, Rotation2d.FromDegrees(30) + Rotation2d.FromDegrees(60) =
     * Rotation2d{-pi/2}
     *
     * @param other The rotation to add.
     *
     * @return The sum of the two rotations.
     */
    Rotation2d operator+(const Rotation2d& other) const;
  
    /**
     * Adds a rotation to the current rotation.
     *
     * This is similar to the + operator except that it mutates the current
     * object.
     *
     * @param other The rotation to add.
     *
     * @return The reference to the new mutated object.
     */
    Rotation2d& operator+=(const Rotation2d& other);
  
    /**
     * Subtracts the new rotation from the current rotation and returns the new
     * rotation.
     *
     * For example, Rotation2d.FromDegrees(10) - Rotation2d.FromDegrees(100) =
     * Rotation2d{-pi/2}
     *
     * @param other The rotation to subtract.
     *
     * @return The difference between the two rotations.
     */
    Rotation2d operator-(const Rotation2d& other) const;
  
    /**
     * Subtracts the new rotation from the current rotation.
     *
     * This is similar to the - operator except that it mutates the current
     * object.
     *
     * @param other The rotation to subtract.
     *
     * @return The reference to the new mutated object.
     */
    Rotation2d& operator-=(const Rotation2d& other);
  
    /**
     * Takes the inverse of the current rotation. This is simply the negative of
     * the current angular value.
     *
     * @return The inverse of the current rotation.
     */
    Rotation2d operator-() const;
  
    /**
     * Multiplies the current rotation by a scalar.
     * @param scalar The scalar.
     *
     * @return The new scaled Rotation2d.
     */
    Rotation2d operator*(double scalar) const;
  
    /**
     * Checks equality between this Rotation2d and another object.
     *
     * @param other The other object.
     * @return Whether the two objects are equal.
     */
    bool operator==(const Rotation2d& other) const;
  
    /**
     * Checks inequality between this Rotation2d and another object.
     *
     * @param other The other object.
     * @return Whether the two objects are not equal.
     */
    bool operator!=(const Rotation2d& other) const;
  
    /**
     * Adds the new rotation to the current rotation using a rotation matrix.
     *
     * [cos_new]   [other.cos, -other.sin][cos]
     * [sin_new] = [other.sin,  other.cos][sin]
     *
     * value_new = std::atan2(cos_new, sin_new)
     *
     * @param other The rotation to rotate by.
     *
     * @return The new rotated Rotation2d.
     */
    Rotation2d rotateBy(const Rotation2d& other) const;
  
    /**
     * Returns the angle of the rotation.
     *
     * @return The angle of the rotation.
     */
    okapi::QAngle angle() const { return m_value; }
  
    /**
     * Returns the cosine of the rotation.
     *
     * @return The cosine of the rotation.
     */
    double cos() const { return m_cos; }
  
    /**
     * Returns the sine of the rotation.
     *
     * @return The sine of the rotation.
     */
    double sin() const { return m_sin; }
  
    /**
     * Returns the tangent of the rotation.
     *
     * @return The tangent of the rotation.
     */
    double tan() const { return m_sin / m_cos; }

private:
    okapi::QAngle m_value = 0_deg;
    double m_cos = 1;
    double m_sin = 0;
};
