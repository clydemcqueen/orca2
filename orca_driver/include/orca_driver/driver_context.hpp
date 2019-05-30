#ifndef ORCA_DRIVER_DRIVER_CONTEXT_HPP
#define ORCA_DRIVER_DRIVER_CONTEXT_HPP

#include <math.h>
#include <string>

#include "orca_driver/context_macros.hpp"

namespace rclcpp
{
  class Node;
}

namespace orca_driver
{

#define CXT_MACRO_ALL_PARAMS \
  CXT_ELEM(num_thrusters, 6, int)                         /* Number of thrusters */ \
  CXT_ELEM(lights_channel, 8, int)                        /* PWM lights channel */ \
  CXT_ELEM(tilt_channel, 9, int)                          /* PWM tilt channel */ \
  CXT_ELEM(voltage_channel, 11, int)                      /* Analog voltage channel */ \
  CXT_ELEM(leak_channel, 12, int)                         /* Digital leak channel */ \
  CXT_ELEM(maestro_port, "/dev/ttyACM0", std::string)     /* Default Maestro port */ \
  CXT_ELEM(voltage_multiplier, 4.7, double)               /* Voltage multiplier */ \
  CXT_ELEM(voltage_min, 12.0, double)                     /* Minimum voltage to run  */ \
/* End of list */

  struct DriverContext
  {
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_FIELD_DEF(n, a)
    CXT_MACRO_ALL_PARAMS

    void load_parameters(rclcpp::Node &node);
  };

} // namespace orca_driver

#endif // ORCA_DRIVER_DRIVER_CONTEXT_HPP
