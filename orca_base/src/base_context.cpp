#include "orca_base/base_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace orca_base {

void BaseContext::load_parameters(rclcpp::Node &node)
{
#undef CXT_ELEM
#define CXT_ELEM(n, a...) CXT_PARAM_LOAD_PARAM(n, a)
  CXT_MACRO_ALL_PARAMS
}

} // namespace orca_base
