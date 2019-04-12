#include "orca_base/trajectory.hpp"

namespace orca_base {

//=====================================================================================
// Utilities
//=====================================================================================

// Compute acceleration required to counteract drag force in the xy plane
void drag_force_to_accel_xy(const double yaw, const double x_v, const double y_v, double &x_a, double &y_a)
{
  // Rotate velocity into the body frame
  double forward_v, strafe_v;
  rotate_frame(x_v, y_v, yaw, forward_v, strafe_v);

  // Calc acceleration due to drag force
  double forward_a = force_to_accel_xy(-drag_force_x(forward_v));
  double strafe_a = force_to_accel_xy(-drag_force_y(strafe_v));

  // Rotate back
  rotate_frame(forward_a, strafe_a, -yaw, x_a, y_a);
}

// Compute the deceleration (glide) distance
double deceleration_distance_xy(const double yaw, double velo_x, double velo_y)
{
  constexpr double dt = 0.1;
  double x = 0;
  double y = 0;

  for (double t = 0; t < 10; t += dt)
  {
    // Compute drag force
    double accel_drag_x, accel_drag_y;
    drag_force_to_accel_xy(yaw, velo_x, velo_y, accel_drag_x, accel_drag_y);

    // Update velocity
    velo_x -= accel_drag_x * dt;
    velo_y -= accel_drag_y * dt;

    // Update distance
    x += velo_x * dt;
    y += velo_y * dt;

    if (std::hypot(velo_x, velo_y) < 0.1)
    {
      // Close enough
      return std::hypot(x, y);
    }
  }

  return 0;
}

//=====================================================================================
// Trajectory
//=====================================================================================

void Trajectory::init(const PoseStamped &p1, const PoseStamped &p2)
{
  p1_ = p1;
  p2_ = p2;

  auto d = (p2.t - p1.t).seconds();
  assert(d > 0);

  // Assume constant velocity
  constant_v_.x = (p2.pose.x - p1.pose.x) / d;
  constant_v_.y = (p2.pose.y - p1.pose.y) / d;
  constant_v_.z = (p2.pose.z - p1.pose.z) / d;
  constant_v_.yaw = norm_angle(p2.pose.yaw - p1.pose.yaw) / d;
}

const State Trajectory::get_state(const rclcpp::Time &t)
{
  if (t > p2_.t) {

    // Hover at p2
    Feedforward ff;
    ff.z = HOVER_ACCEL_Z;

    return State{t, p2_.pose, Twist{}, ff};

  } else {

    auto d = (t - p1_.t).seconds();

    // Pose at t
    Pose p;
    p.x = p1_.pose.x + constant_v_.x * d;
    p.y = p1_.pose.y + constant_v_.y * d;
    p.z = p1_.pose.z + constant_v_.z * d;
    p.yaw = norm_angle(p1_.pose.yaw + constant_v_.yaw * d);

    // Acceleration to counter drag and buoyancy at a constant velocity
    Feedforward ff;
    drag_force_to_accel_xy(p.yaw, constant_v_.x, constant_v_.y, ff.x, ff.y);
    ff.z = force_to_accel_z(-drag_force_z(constant_v_.z)) + HOVER_ACCEL_Z;
    ff.yaw = torque_to_accel_yaw(-drag_torque_yaw(constant_v_.yaw));

    return State{t, p, constant_v_, ff};
  }
}

} // namespace orca_base