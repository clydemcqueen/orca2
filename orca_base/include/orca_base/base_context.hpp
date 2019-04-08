#ifndef ORCA_BASE_BASE_CONTEXT_HPP
#define ORCA_BASE_BASE_CONTEXT_HPP

#include "orca_base/context_macros.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rclcpp {
  class Node;
}

namespace orca_base {

#define CXT_MACRO_ALL_PARAMS \
  CXT_ELEM(inc_yaw, 0.2, double)                                        /* Yaw trim increment  */ \
  CXT_ELEM(inc_z, 0.2, double)                                          /* Z trim increment  */ \
  CXT_ELEM(inc_tilt, 5, int)                                            /* Tilt increment  */ \
  CXT_ELEM(inc_lights, 20, int)                                         /* Lights increment  */ \
  \
  CXT_ELEM(input_dead_band, 0.05f, float)                               /* Ignore small joystick inputs  */ \
  CXT_ELEM(yaw_pid_dead_band, 0.0005, double)                           /* Ignore small yaw pid outputs  */ \
  CXT_ELEM(z_pid_dead_band, 0.002, double)                              /* Ignore small z pid outputs  */ \
  \
  CXT_ELEM(xy_gain, 0.5, double)                                        /* Attenuate joystick inputs  */ \
  CXT_ELEM(yaw_gain, 0.2, double)                                       /* Attenuate joystick inputs  */ \
  CXT_ELEM(vertical_gain, 0.5, double)                                  /* Attenuate joystick inputs  */ \
  /* End of list */

// TODO add map_frame_id_ "map"
// TODO add base_frame_id_ "base_link"
// TODO add z_controller kp, ki, kd
// TODO add yaw_controller kp, ki, kd

struct BaseContext
{
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_FIELD_DEF(n, a)
  CXT_MACRO_ALL_PARAMS

  void load_parameters(rclcpp::Node &node);
};

} // namespace orca_base

#endif // ORCA_BASE_BASE_CONTEXT_HPP
