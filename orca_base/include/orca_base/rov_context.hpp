#ifndef ORCA_BASE_ROV_CONTEXT_HPP
#define ORCA_BASE_ROV_CONTEXT_HPP

#include <cmath>
#include <string>
#include <vector>

#include "ros2_shared/context_macros.hpp"

#include "orca_shared/model.hpp"

namespace orca_base
{

#define ROV_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(bollard_force_z_up, double, 100)           /* Newtons, bollard force z, moving up  */ \
  CXT_MACRO_MEMBER(bollard_force_z_down, double, 80)          /* Newtons, bollard force z, moving down  */ \
  CXT_MACRO_MEMBER(fluid_density, double, 997)                /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
  \
  CXT_MACRO_MEMBER(base_frame, std::string, "base_link")      /* Base frame  */ \
  \
  CXT_MACRO_MEMBER(inc_pressure, double, 2000)                /* Pressure trim increment  */ \
  CXT_MACRO_MEMBER(inc_tilt, int, 5)                          /* Tilt increment  */ \
  CXT_MACRO_MEMBER(inc_lights, int, 20)                       /* Lights increment  */ \
  \
  CXT_MACRO_MEMBER(input_dead_band, float, 0.05f)             /* Ignore small joystick inputs  */ \
  CXT_MACRO_MEMBER(xy_limit, double, 0.5)                     /* Attenuate joystick inputs  */ \
  CXT_MACRO_MEMBER(yaw_gain, double, 0.2)                     /* Attenuate joystick inputs  */ \
  CXT_MACRO_MEMBER(vertical_gain, double, 0.5)                /* Attenuate joystick inputs  */ \
  \
  CXT_MACRO_MEMBER(rov_pressure_pid_kp, double, 0.00024)      /* ROV hold pressure pid Kp  */ \
  CXT_MACRO_MEMBER(rov_pressure_pid_ki, double, 0.00015)      /* ROV hold pressure pid Ki  */ \
  CXT_MACRO_MEMBER(rov_pressure_pid_kd, double, 0.000096)     /* ROV hold pressure pid Kd  */ \
  \
  CXT_MACRO_MEMBER(planner_target_z, double, -0.5)            /* AUV path target z position  */ \
  \
  CXT_MACRO_MEMBER(timeout_baro_ms, int, 400)                 /* Barometer message timeout in ms  */ \
  CXT_MACRO_MEMBER(timeout_joy_ms, int, 1000)                 /* Joy message timeout in ms  */ \
  CXT_MACRO_MEMBER(timer_period_ms, int, 50)                  /* Timer period in ms  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct ROVContext
  {
    ROV_NODE_ALL_PARAMS

    // Orca model
    orca::Model model_{};
  };

} // namespace orca_base

#endif //ORCA_BASE_ROV_CONTEXT_HPP
