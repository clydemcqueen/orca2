#ifndef ORCA_BASE_UTIL_HPP
#define ORCA_BASE_UTIL_HPP

#include <cstdint>
#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Transform.h"

namespace orca_base
{

  template<typename T>
  constexpr T clamp(const T v, const T min, const T max)
  {
    return v > max ? max : (v < min ? min : v);
  }

  template<typename T>
  constexpr T clamp(const T v, const T minmax)
  {
    return clamp(v, -minmax, minmax);
  }

  template<typename T>
  constexpr T dead_band(const T v, const T d)
  {
    return v < d && v > -d ? 0 : v;
  }

  template<typename A, typename B>
  constexpr B scale(const A a, const A a_min, const A a_max, const B b_min, const B b_max)
  {
    return clamp(static_cast<B>(b_min + static_cast<double>(b_max - b_min) / (a_max - a_min) * (a - a_min)), b_min,
                 b_max);
  }

#if 1

// Move an angle to the region [-M_PI, M_PI)
  constexpr double norm_angle(double a)
  {
    if (a < -M_PI || a > M_PI) {
      // Force to [-2PI, 2PI)
      a = fmod(a, 2 * M_PI);

      // Move to [-PI, PI)
      if (a < -M_PI) {
        a += 2 * M_PI;
      } else if (a > M_PI) {
        a -= 2 * M_PI;
      }
    }

    return a;
  }

#else
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
#endif

  // Compute a 2d point in a rotated frame (v' = R_transpose * v)
  void rotate_frame(double x, double y, double theta, double &x_r, double &y_r);

  // Get roll, pitch and yaw from a quaternion
  void get_rpy(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw);

  // Get yaw from a quaternion
  double get_yaw(const geometry_msgs::msg::Quaternion &q);

  // Sense a button down event
  bool button_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int button);

  bool trim_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int axis);

  // Various to_str functions
  std::string to_str_rpy(const tf2::Transform &t);

  std::string to_str_q(const tf2::Transform &t);

  std::string to_str(const rclcpp::Time &t);

  std::string to_str(const builtin_interfaces::msg::Time &t);

} // namespace orca_base

#endif // ORCA_BASE_UTIL_HPP
