#include "orca_base/model.hpp"

namespace orca_base {

// Compute acceleration required to counteract drag force
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
double deceleration_distance(const double yaw, double velo_x, double velo_y)
{
  constexpr double dt = 0.1;
  double x = 0;
  double y = 0;

  for (double t = 0; t < 10; t += dt) {
    // Compute drag force
    double accel_drag_x, accel_drag_y;
    drag_force_to_accel_xy(yaw, velo_x, velo_y, accel_drag_x, accel_drag_y);

    // Update velocity
    velo_x -= accel_drag_x * dt;
    velo_y -= accel_drag_y * dt;

    // Update distance
    x += velo_x * dt;
    y += velo_y * dt;

    if (std::hypot(velo_x, velo_y) < 0.1) {
      // Close enough
      return std::hypot(x, y);
    }
  }

  return 0;
}

} // namespace orca_base