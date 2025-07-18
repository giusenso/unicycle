#pragma once
#include <cmath>

/**
 * @brief Converts radians to degrees.
 *
 * @param rad Angle in radians.
 * @return Angle in degrees.
 */
inline double radToDeg(double rad) { return rad * (180.0 / M_PI); }

/**
 * @brief Converts degrees to radians.
 *
 * @param deg Angle in degrees.
 * @return Angle in radians.
 */
inline double degToRad(double deg) { return deg * (M_PI / 180.0); }

/**
 * @brief Compute a + b wrapped to [0, 2π)
 *
 * @param a First angle in radians.
 * @param b Second angle in radians.
 * @return double The wrapped sum in [0, 2π).
 */
inline double angularSum(double a, double b) {
  double sum = std::fmod(a + b, 2 * M_PI);
  if (sum < 0)
    sum += 2 * M_PI;
  return sum;
}

/**
 * @brief Compute a - b wrapped to [0, 2π)
 *
 * @param a First angle in radians.
 * @param b Second angle in radians.
 * @return double The wrapped difference in [0, 2π).
 */
inline double angularDiff(double a, double b) {
  double diff = std::fmod(a - b, 2 * M_PI);
  if (diff < 0)
    diff += 2 * M_PI;
  return diff;
}
