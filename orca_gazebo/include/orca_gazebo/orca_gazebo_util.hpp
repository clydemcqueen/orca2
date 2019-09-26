#ifndef ORCA_GAZEBO_UTIL_H
#define ORCA_GAZEBO_UTIL_H

#include "ignition/math/Vector3.hh"
#include "ignition/math/Quaternion.hh"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace orca_gazebo
{
  std::default_random_engine k_generator;

  // Assume x, y and z are independent, e.g., MEMS accelerometers or gyros
  void addNoise(const double stddev, ignition::math::Vector3d &v)
  {
    std::normal_distribution<double> distribution(0, stddev);

    v.X() += distribution(k_generator);
    v.Y() += distribution(k_generator);
    v.Z() += distribution(k_generator);
  }

// Assume r, p and y are independent, e.g., MEMS magnetometers
  void addNoise(const double stddev, ignition::math::Quaterniond &q)
  {
    ignition::math::Vector3d v = q.Euler();
    addNoise(stddev, v);
    q.Euler(v);
  }

  void ignition2msg(const ignition::math::Vector3d &i, geometry_msgs::msg::Vector3 &m)
  {
    m.x = i.X();
    m.y = i.Y();
    m.z = i.Z();
  }

  void ignition2msg(const ignition::math::Quaterniond &i, geometry_msgs::msg::Quaternion &m)
  {
    m.x = i.X();
    m.y = i.Y();
    m.z = i.Z();
    m.w = i.W();
  }

} // namespace orca_gazebo

#endif // ORCA_GAZEBO_UTIL_H
