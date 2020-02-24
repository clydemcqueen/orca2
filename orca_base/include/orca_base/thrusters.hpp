#ifndef ORCA_BASE_THRUSTERS_HPP
#define ORCA_BASE_THRUSTERS_HPP

namespace orca_base
{

  //=============================================================================
  // Thrusters
  //=============================================================================

  struct Thruster
  {
    std::string frame_id;   // URDF link frame id
    bool ccw;               // True if counterclockwise
    double forward_factor;
    double strafe_factor;
    double yaw_factor;
    double vertical_factor;
  };

  // Thrusters, order must match the order of the <thruster> tags in the URDF
  const std::vector<Thruster> THRUSTERS = {
    {"t200_link_front_right",    false, 1.0, 1.0,  1.0,  0.0},
    {"t200_link_front_left",     false, 1.0, -1.0, -1.0, 0.0},
    {"t200_link_rear_right",     true,  1.0, -1.0, 1.0,  0.0},
    {"t200_link_rear_left",      true,  1.0, 1.0,  -1.0, 0.0},
    {"t200_link_vertical_right", false, 0.0, 0.0,  0.0,  1.0},
    {"t200_link_vertical_left",  true,  0.0, 0.0,  0.0,  -1.0},
  };

} // namespace orca_base

#endif // ORCA_BASE_THRUSTERS_HPP
