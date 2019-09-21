#ifndef ORCA_BASE_BASE_CONTEXT_HPP
#define ORCA_BASE_BASE_CONTEXT_HPP

#include <cmath>
#include <string>
#include <vector>

#include "ros2_shared/context_macros.hpp"

namespace orca_base
{

#define BASE_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(auto_start, int, 0)                        /* Auto-start AUV mission if > 0  */ \
  CXT_MACRO_MEMBER(map_frame, std::string, "map")             /* Map frame  */ \
  CXT_MACRO_MEMBER(base_frame, std::string, "base_link")      /* Base frame  */ \
  \
  CXT_MACRO_MEMBER(inc_yaw, double, 0.2)                      /* Yaw trim increment  */ \
  CXT_MACRO_MEMBER(inc_z, double, 0.2)                        /* Z trim increment  */ \
  CXT_MACRO_MEMBER(inc_tilt, int, 5)                          /* Tilt increment  */ \
  CXT_MACRO_MEMBER(inc_lights, int, 20)                       /* Lights increment  */ \
  \
  CXT_MACRO_MEMBER(input_dead_band, float, 0.05f)             /* Ignore small joystick inputs  */ \
  CXT_MACRO_MEMBER(xy_gain, double, 0.5)                      /* Attenuate joystick inputs  */ \
  CXT_MACRO_MEMBER(yaw_gain, double, 0.3)                     /* Attenuate joystick inputs  */ \
  CXT_MACRO_MEMBER(vertical_gain, double, 0.5)                /* Attenuate joystick inputs  */ \
  \
  CXT_MACRO_MEMBER(rov_z_pid_kp, double, 2.4)                 /* ROV z pid Kp  */ \
  CXT_MACRO_MEMBER(rov_z_pid_ki, double, 1.5)                 /* ROV z pid Ki  */ \
  CXT_MACRO_MEMBER(rov_z_pid_kd, double, 0.96)                /* ROV z pid Kd  */ \
  \
  CXT_MACRO_MEMBER(rov_yaw_pid_kp, double, 5)                 /* ROV yaw pid Kp  */ \
  CXT_MACRO_MEMBER(rov_yaw_pid_ki, double, 0)                 /* ROV yaw pid Ki  */ \
  CXT_MACRO_MEMBER(rov_yaw_pid_kd, double, 0)                 /* ROV yaw pid Kd  */ \
  \
  CXT_MACRO_MEMBER(auv_x_pid_kp, double, 3)                   /* AUV x pid Kp  */ \
  CXT_MACRO_MEMBER(auv_x_pid_ki, double, 2)                   /* AUV x pid Ki  */ \
  CXT_MACRO_MEMBER(auv_x_pid_kd, double, 1.125)               /* AUV x pid Kd  */ \
  \
  CXT_MACRO_MEMBER(auv_y_pid_kp, double, 3)                   /* AUV y pid Kp  */ \
  CXT_MACRO_MEMBER(auv_y_pid_ki, double, 2)                   /* AUV y pid Ki  */ \
  CXT_MACRO_MEMBER(auv_y_pid_kd, double, 1.125)               /* AUV y pid Kd  */ \
  \
  CXT_MACRO_MEMBER(auv_z_pid_kp, double, 2.4)                 /* AUV z pid Kp  */ \
  CXT_MACRO_MEMBER(auv_z_pid_ki, double, 1.5)                 /* AUV z pid Ki  */ \
  CXT_MACRO_MEMBER(auv_z_pid_kd, double, 0.96)                /* AUV z pid Kd  */ \
  \
  CXT_MACRO_MEMBER(auv_yaw_pid_kp, double, 5)                 /* AUV yaw pid Kp  */ \
  CXT_MACRO_MEMBER(auv_yaw_pid_ki, double, 0)                 /* AUV yaw pid Ki  */ \
  CXT_MACRO_MEMBER(auv_yaw_pid_kd, double, 0)                 /* AUV yaw pid Kd  */ \
  \
  CXT_MACRO_MEMBER(auv_z_target, double, -0.25)               /* AUV path target z position  */ \
  CXT_MACRO_MEMBER(auv_xy_distance, double, 1)                /* AUV distance in front of marker  */ \
  CXT_MACRO_MEMBER(auv_xy_speed, double, 0.5)                 /* AUV speed in the xy plane  */ \
  CXT_MACRO_MEMBER(auv_z_speed, double, 0.3)                  /* AUV vertical speed  */ \
  CXT_MACRO_MEMBER(auv_yaw_speed, double, M_PI_4 / 2)         /* AUV rotation speed  */ \
  \
  CXT_MACRO_MEMBER(keep_poses, int, 100)                      /* Max # of poses on filtered_path  */ \
  \
  CXT_MACRO_MEMBER(auv_controller, int, 0)                    /* Controller  */ \
  CXT_MACRO_MEMBER(auv_epsilon_xy, double, 0.1)               /* Deadzone controller epison xy  */ \
  CXT_MACRO_MEMBER(auv_epsilon_z, double, 0.1)                /* Deadzone controller epison z  */ \
  CXT_MACRO_MEMBER(auv_epsilon_yaw, double, 0.2)              /* Deadzone controller epison yaw  */ \
  CXT_MACRO_MEMBER(auv_jerk_xy, double, 0.1)                  /* Slow controller jerk xy  */ \
  CXT_MACRO_MEMBER(auv_jerk_z, double, 0.1)                   /* Slow controller jerk z  */ \
  CXT_MACRO_MEMBER(auv_jerk_yaw, double, 0.2)                 /* Slow controller jerk yaw  */ \
  \
  CXT_MACRO_MEMBER(filter_use_output, bool, true)             /* Use filtered_odom instead of raw odom  */ \
  CXT_MACRO_MEMBER(filter_predict_accel, bool, true)          /* Predict acceleration  */ \
  CXT_MACRO_MEMBER(filter_predict_control, bool, true)        /* Add u_bar to predicted acceleration  */ \
  CXT_MACRO_MEMBER(filter_predict_drag, bool, true)           /* Add drag to predicted acceleration  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct BaseContext
  {
    BASE_NODE_ALL_PARAMS
  };

} // namespace orca_base

#endif // ORCA_BASE_BASE_CONTEXT_HPP
