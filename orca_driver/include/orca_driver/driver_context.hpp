#ifndef ORCA_DRIVER_DRIVER_CONTEXT_HPP
#define ORCA_DRIVER_DRIVER_CONTEXT_HPP

#include <math.h>
#include <string>

#include "ros2_shared/context_macros.hpp"

namespace orca_driver
{

#define DRIVER_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(timer_period_ms, int, 100)                     /* Timer period in ms  */ \
  CXT_MACRO_MEMBER(timeout_control_ms, int, 1000)                 /* Control msg timeout */ \
  \
  CXT_MACRO_MEMBER(num_thrusters, int, 6)                         /* Number of thrusters */ \
  CXT_MACRO_MEMBER(lights_channel, int, 8)                        /* PWM lights channel */ \
  CXT_MACRO_MEMBER(tilt_channel, int, 9)                          /* PWM tilt channel */ \
  CXT_MACRO_MEMBER(voltage_channel, int, 11)                      /* Analog voltage channel */ \
  CXT_MACRO_MEMBER(leak_channel, int, 12)                         /* Digital leak channel */ \
  CXT_MACRO_MEMBER(maestro_port, std::string, "/dev/ttyACM0")     /* Default Maestro port */ \
  CXT_MACRO_MEMBER(voltage_multiplier, double, 4.7)               /* Voltage multiplier */ \
  CXT_MACRO_MEMBER(voltage_min, double, 14.0)                     /* Minimum voltage to run  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct DriverContext
  {
    DRIVER_NODE_ALL_PARAMS
  };

} // namespace orca_driver

#endif // ORCA_DRIVER_DRIVER_CONTEXT_HPP
