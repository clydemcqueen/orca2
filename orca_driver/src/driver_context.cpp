#include "orca_driver/driver_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace orca_driver
{

  void DriverContext::load_parameters(rclcpp::Node &node)
  {
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_LOAD_PARAM(n, a)
    CXT_MACRO_ALL_PARAMS
  }

} // namespace orca_driver
