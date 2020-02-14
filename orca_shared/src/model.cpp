#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"

namespace orca
{

  void Model::linear_drag_world(double yaw, double motion_world, double &linear_drag_x, double &linear_drag_y)
  {
    // Direction of motion in the body frame
    auto motion_body = norm_angle(motion_world - yaw);

    // Fold quadrants II, II and IV into quadrant I
    if (motion_body < 0) {
      motion_body = -motion_body;
    }
    if (motion_body > M_PI / 2) {
      motion_body = M_PI - motion_body;
    }

    // Interpolate between linear_drag_s and linear_drag_f to find the drag constant for the direction of motion
    auto linear_drag_motion = (motion_body * linear_drag_s() + (M_PI / 2 - motion_body) * linear_drag_f()) / (M_PI / 2);

    // Break the drag down to x and y components
    // Coef must be positive, note abs()
    linear_drag_x = abs(cos(motion_world)) * linear_drag_motion;
    linear_drag_y = abs(sin(motion_world)) * linear_drag_motion;
  }

} // namespace orca