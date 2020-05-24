#ifndef ORCA_DRIVER_DRIVER_CONTEXT_HPP
#define ORCA_DRIVER_DRIVER_CONTEXT_HPP

#include <cmath>
#include <string>

#include "ros2_shared/context_macros.hpp"

namespace orca_driver
{

  // See BlueROV2 thruster diagram: https://bluerobotics.com/learn/bluerov2-assembly/

#define DRIVER_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(timer_period_ms, int, 100)                     /* Timer period in ms  */ \
  CXT_MACRO_MEMBER(timeout_control_ms, int, 1000)                 /* Control msg timeout */ \
  \
  CXT_MACRO_MEMBER(thruster_1_channel, int, 4)                    /* Front right */ \
  CXT_MACRO_MEMBER(thruster_2_channel, int, 7)                    /* Front left */ \
  CXT_MACRO_MEMBER(thruster_3_channel, int, 0)                    /* Rear right */ \
  CXT_MACRO_MEMBER(thruster_4_channel, int, 5)                    /* Rear left */ \
  CXT_MACRO_MEMBER(thruster_5_channel, int, 1)                    /* Vertical right */ \
  CXT_MACRO_MEMBER(thruster_6_channel, int, 6)                    /* Vertical left */ \
  \
  CXT_MACRO_MEMBER(thruster_1_reverse, bool, false) \
  CXT_MACRO_MEMBER(thruster_2_reverse, bool, false) \
  CXT_MACRO_MEMBER(thruster_3_reverse, bool, false) \
  CXT_MACRO_MEMBER(thruster_4_reverse, bool, false) \
  CXT_MACRO_MEMBER(thruster_5_reverse, bool, false) \
  CXT_MACRO_MEMBER(thruster_6_reverse, bool, false) \
  \
  CXT_MACRO_MEMBER(lights_channel, int, 9)                        /* PWM lights channel */ \
  CXT_MACRO_MEMBER(tilt_channel, int, 10)                         /* PWM tilt channel */ \
  CXT_MACRO_MEMBER(voltage_channel, int, 11)                      /* Analog voltage channel */ \
  CXT_MACRO_MEMBER(leak_channel, int, 12)                         /* Digital leak channel */ \
  CXT_MACRO_MEMBER(maestro_port, std::string, "/dev/ttyACM0")     /* Default Maestro port */ \
  CXT_MACRO_MEMBER(voltage_multiplier, double, 5.52)              /* Voltage multiplier */ \
  CXT_MACRO_MEMBER(voltage_min, double, 12.0)                     /* Minimum voltage to run  */ \
  \
  CXT_MACRO_MEMBER(read_battery, bool, true)                      /* Read voltage sensor  */ \
  CXT_MACRO_MEMBER(read_leak, bool, true)                         /* Read leak sensor  */ \
  CXT_MACRO_MEMBER(read_temp, bool, true)                         /* Read temp file  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct DriverContext
  {
    DRIVER_NODE_ALL_PARAMS
  };

} // namespace orca_driver

#endif // ORCA_DRIVER_DRIVER_CONTEXT_HPP
