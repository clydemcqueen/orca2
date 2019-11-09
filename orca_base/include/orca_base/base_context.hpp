#ifndef ORCA_BASE_BASE_CONTEXT_HPP
#define ORCA_BASE_BASE_CONTEXT_HPP

#include <cmath>
#include <string>
#include <vector>

#include "ros2_shared/context_macros.hpp"

#include "orca_base/model.hpp"

namespace orca_base
{

  struct Controllers
  {
    static constexpr int SIMPLE = 0;
    static constexpr int IGNORE_ESTIMATE = 1;
    static constexpr int DEADZONE = 2;
    static constexpr int JERK = 3;
    static constexpr int BEST = 4;
    static constexpr int DEPTH = 5;
  };

#define BASE_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(param_fluid_density, double, 997)          /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
  \
  CXT_MACRO_MEMBER(auto_start, int, 0)                        /* Auto-start AUV mission if > 0  */ \
  CXT_MACRO_MEMBER(map_frame, std::string, "map")             /* Map frame  */ \
  CXT_MACRO_MEMBER(base_frame, std::string, "base_link")      /* Base frame  */ \
  \
  CXT_MACRO_MEMBER(inc_pressure, double, 2000)                /* Pressure trim increment  */ \
  CXT_MACRO_MEMBER(inc_tilt, int, 5)                          /* Tilt increment  */ \
  CXT_MACRO_MEMBER(inc_lights, int, 20)                       /* Lights increment  */ \
  \
  CXT_MACRO_MEMBER(input_dead_band, float, 0.05f)             /* Ignore small joystick inputs  */ \
  CXT_MACRO_MEMBER(xy_gain, double, 0.5)                      /* Attenuate joystick inputs  */ \
  CXT_MACRO_MEMBER(yaw_gain, double, 0.3)                     /* Attenuate joystick inputs  */ \
  CXT_MACRO_MEMBER(vertical_gain, double, 0.5)                /* Attenuate joystick inputs  */ \
  \
  CXT_MACRO_MEMBER(rov_pressure_pid_kp, double, 0.00024)      /* ROV hold pressure pid Kp  */ \
  CXT_MACRO_MEMBER(rov_pressure_pid_ki, double, 0.00015)      /* ROV hold pressure pid Ki  */ \
  CXT_MACRO_MEMBER(rov_pressure_pid_kd, double, 0.000096)     /* ROV hold pressure pid Kd  */ \
  \
  CXT_MACRO_MEMBER(auv_x_pid_ku, double, 1)                   /* AUV x pid Ziegler–Nichols ku  */ \
  CXT_MACRO_MEMBER(auv_x_pid_tu, double, 6)                   /* AUV x pid Ziegler–Nichols tu  */ \
  \
  CXT_MACRO_MEMBER(auv_y_pid_ku, double, 1)                   /* AUV y pid Ziegler–Nichols ku  */ \
  CXT_MACRO_MEMBER(auv_y_pid_tu, double, 6)                   /* AUV y pid Ziegler–Nichols tu  */ \
  \
  CXT_MACRO_MEMBER(auv_z_pid_ku, double, 1)                   /* AUV z pid Ziegler–Nichols ku  */ \
  CXT_MACRO_MEMBER(auv_z_pid_tu, double, 6)                   /* AUV z pid Ziegler–Nichols tu  */ \
  \
  CXT_MACRO_MEMBER(auv_yaw_pid_ku, double, 1)                 /* AUV yaw pid Ziegler–Nichols ku  */ \
  CXT_MACRO_MEMBER(auv_yaw_pid_tu, double, 6)                 /* AUV yaw pid Ziegler–Nichols tu  */ \
  \
  CXT_MACRO_MEMBER(auv_z_target, double, -0.25)               /* AUV path target z position  */ \
  CXT_MACRO_MEMBER(auv_xy_distance, double, 1)                /* AUV distance in front of marker  */ \
  CXT_MACRO_MEMBER(auv_xy_speed, double, 0.5)                 /* AUV speed in the xy plane  */ \
  CXT_MACRO_MEMBER(auv_z_speed, double, 0.3)                  /* AUV vertical speed  */ \
  CXT_MACRO_MEMBER(auv_yaw_speed, double, M_PI_4 / 2)         /* AUV rotation speed  */ \
  \
  CXT_MACRO_MEMBER(keep_poses, int, 500)                      /* Max # of poses on filtered_path  */ \
  \
  CXT_MACRO_MEMBER(auv_controller, int, 0)                    /* Controller, type Controllers  */ \
  CXT_MACRO_MEMBER(auv_epsilon_xy, double, 0.1)               /* Deadzone controller epsilon xy  */ \
  CXT_MACRO_MEMBER(auv_epsilon_z, double, 0.1)                /* Deadzone controller epsilon z  */ \
  CXT_MACRO_MEMBER(auv_epsilon_yaw, double, 0.2)              /* Deadzone controller epsilon yaw  */ \
  CXT_MACRO_MEMBER(auv_jerk_xy, double, 0.1)                  /* Slow controller jerk xy  */ \
  CXT_MACRO_MEMBER(auv_jerk_z, double, 0.1)                   /* Slow controller jerk z  */ \
  CXT_MACRO_MEMBER(auv_jerk_yaw, double, 0.2)                 /* Slow controller jerk yaw  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct BaseContext
  {
    BASE_NODE_ALL_PARAMS

    // Orca model
    Model model_{};
  };

} // namespace orca_base

#endif // ORCA_BASE_BASE_CONTEXT_HPP
