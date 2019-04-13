#ifndef ORCA_BASE_UTIL_HPP
#define ORCA_BASE_UTIL_HPP

#include <cstdint>
#include <math.h>

#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace orca_base {

template<typename T>
constexpr const T clamp(const T v, const T min, const T max)
{
  return v > max ? max : (v < min ? min : v);
}

template<typename T>
constexpr const T dead_band(const T v, const T d)
{
  return v < d && v > -d ? 0 : v;
}

template<typename A, typename B>
constexpr const B scale(const A a, const A a_min, const A a_max, const B b_min, const B b_max)
{
  return clamp(static_cast<B>(b_min + static_cast<double>(b_max - b_min) / (a_max - a_min) * (a - a_min)), b_min, b_max);
}

// Move an angle to the region [-M_PI, M_PI]
constexpr double norm_angle(double a)
{
  while (a < -M_PI) {
    a += 2 * M_PI;
  }
  while (a > M_PI) {
    a -= 2 * M_PI;
  }

  return a;
}

// Compute a 2d point in a rotated frame (v' = R_transpose * v)
void rotate_frame(const double x, const double y, const double theta, double &x_r, double &y_r);

// Get yaw from a quaternion
double get_yaw(const geometry_msgs::msg::Quaternion &q);

// Sense a button down event
bool button_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int button);
bool trim_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int axis);

} // namespace orca_base

#endif // ORCA_BASE_UTIL_HPP
