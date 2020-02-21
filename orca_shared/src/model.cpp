#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"

namespace orca
{

  void Model::drag_const_world(double yaw, double motion_world, double &drag_const_x, double &drag_const_y)
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

    // Interpolate between drag_const_s and drag_const_f to find the drag constant for the direction of motion
    auto drag_const_motion = (motion_body * drag_const_s() + (M_PI / 2 - motion_body) * drag_const_f()) / (M_PI / 2);

    // Break the drag down to x and y components
    // Coef must be positive, note abs()
    drag_const_x = abs(cos(motion_world)) * drag_const_motion;
    drag_const_y = abs(sin(motion_world)) * drag_const_motion;
  }

} // namespace orca