#ifndef ORCA_BASE_TRAJECTORY_HPP
#define ORCA_BASE_TRAJECTORY_HPP

#include "orca_base/geometry.hpp"

namespace orca_base {

class Trajectory
{
  PoseStamped p1_;
  PoseStamped p2_;
  Twist constant_v_;

public:

  const PoseStamped &p1() const { return p1_; }
  const PoseStamped &p2() const { return p2_; }

  // Model a trajectory from p1 to p2
  void init(const PoseStamped &p1, const PoseStamped &p2);

  // Return projected state at time t
  const State get_state(const rclcpp::Time &t);
};

} // namespace orca_base

#endif //ORCA_BASE_TRAJECTORY_HPP
