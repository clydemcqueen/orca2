#ifndef ORCA_BASE_BASE_CONTEXT_HPP
#define ORCA_BASE_BASE_CONTEXT_HPP

#include "orca_base/context_macros.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rclcpp {
  class Node;
}

namespace orca_base {

#define CXT_MACRO_ALL_PARAMS \
  CXT_ELEM(simulation, true, bool)                                      /* We're in a simulation  */ \
  CXT_ELEM(map_frame, "map", std::string)                               /* Map frame  */ \
  CXT_ELEM(base_frame, "base_link", std::string)                        /* Base frame  */ \
  \
  CXT_ELEM(inc_yaw, 0.2, double)                                        /* Yaw trim increment  */ \
  CXT_ELEM(inc_z, 0.2, double)                                          /* Z trim increment  */ \
  CXT_ELEM(inc_tilt, 5, int)                                            /* Tilt increment  */ \
  CXT_ELEM(inc_lights, 20, int)                                         /* Lights increment  */ \
  \
  CXT_ELEM(input_dead_band, 0.05f, float)                               /* Ignore small joystick inputs  */ \
  CXT_ELEM(xy_gain, 0.5, double)                                        /* Attenuate joystick inputs  */ \
  CXT_ELEM(yaw_gain, 0.2, double)                                       /* Attenuate joystick inputs  */ \
  CXT_ELEM(vertical_gain, 0.5, double)                                  /* Attenuate joystick inputs  */ \
  \
  CXT_ELEM(rov_z_pid_kp, 0.1, double)                                   /* ROV z pid Kp  */ \
  CXT_ELEM(rov_z_pid_ki, 0, double)                                     /* ROV z pid Ki  */ \
  CXT_ELEM(rov_z_pid_kd, 0.05, double)                                  /* ROV z pid Kd  */ \
  \
  CXT_ELEM(rov_yaw_pid_kp, 0.05, double)                                /* ROV yaw pid Kp  */ \
  CXT_ELEM(rov_yaw_pid_ki, 0, double)                                   /* ROV yaw pid Ki  */ \
  CXT_ELEM(rov_yaw_pid_kd, 0, double)                                   /* ROV yaw pid Kd  */ \
  \
  CXT_ELEM(auv_x_pid_kp, 1, double)                                     /* AUV x pid Kp  */ \
  CXT_ELEM(auv_x_pid_ki, 0, double)                                     /* AUV x pid Ki  */ \
  CXT_ELEM(auv_x_pid_kd, 0, double)                                     /* AUV x pid Kd  */ \
  \
  CXT_ELEM(auv_y_pid_kp, 1, double)                                     /* AUV y pid Kp  */ \
  CXT_ELEM(auv_y_pid_ki, 0, double)                                     /* AUV y pid Ki  */ \
  CXT_ELEM(auv_y_pid_kd, 0, double)                                     /* AUV y pid Kd  */ \
  \
  CXT_ELEM(auv_z_pid_kp, 2, double)                                     /* AUV z pid Kp  */ \
  CXT_ELEM(auv_z_pid_ki, 0.05, double)                                  /* AUV z pid Ki  */ \
  CXT_ELEM(auv_z_pid_kd, 0, double)                                     /* AUV z pid Kd  */ \
  \
  CXT_ELEM(auv_yaw_pid_kp, 0.5, double)                                 /* AUV yaw pid Kp  */ \
  CXT_ELEM(auv_yaw_pid_ki, 0, double)                                   /* AUV yaw pid Ki  */ \
  CXT_ELEM(auv_yaw_pid_kd, 0, double)                                   /* AUV yaw pid Kd  */ \
  /* End of list */

struct BaseContext
{
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_FIELD_DEF(n, a)
  CXT_MACRO_ALL_PARAMS

  void load_parameters(rclcpp::Node &node);
};

} // namespace orca_base

#endif // ORCA_BASE_BASE_CONTEXT_HPP
