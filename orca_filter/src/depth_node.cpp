// Copyright (c) 2020, Clyde McQueen.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>
#include <string>

#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_shared/baro.hpp"
#include "orca_shared/monotonic.hpp"
#include "rclcpp/node.hpp"
#include "ros2_shared/context_macros.hpp"

namespace orca_filter
{

//=============================================================================
// Parameter(s)
//=============================================================================

#define DEPTH_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(fluid_density, double, 997) /* kg/m^3, 997 freshwater, 1029 seawater  */ \
  CXT_MACRO_MEMBER(frame_id, std::string, "map") \
  CXT_MACRO_MEMBER(z_variance, double, orca::Model::DEPTH_STDDEV * orca::Model::DEPTH_STDDEV * 10) \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct DepthContext
{
  DEPTH_NODE_ALL_PARAMS

  // Orca model
  orca::Model model_{};
};

//=============================================================================
// DepthNode subscribes to /barometer (air + water pressure at baro_link)
// and publishes /depth (base_link.z in the map frame)
//=============================================================================

constexpr int QUEUE_SIZE = 10;

class DepthNode : public rclcpp::Node
{
  DepthContext cxt_;                  // Parameter(s)
  orca::Barometer barometer_{};       // Barometer state

  rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Publisher<orca_msgs::msg::Depth>::SharedPtr depth_pub_;

  // Callback wrapper, guarantees timestamp monotonicity
  monotonic::Monotonic<DepthNode *, const orca_msgs::msg::Barometer::SharedPtr>
  baro_cb_{this, &DepthNode::baro_callback};

  // Barometer callback
  void baro_callback(orca_msgs::msg::Barometer::SharedPtr baro_msg, bool first)
  {
    orca_msgs::msg::Depth depth_msg;
    depth_msg.header.stamp = baro_msg->header.stamp;
    depth_msg.header.frame_id = cxt_.frame_id_;

    // Convert pressure at baro_link to depth at base_link
    depth_msg.z = barometer_.pressure_to_base_link_z(cxt_.model_, baro_msg->pressure);

    // Measurement uncertainty
    depth_msg.z_variance = cxt_.z_variance_;

    depth_pub_->publish(depth_msg);
  }

  // Validate parameters
  void validate_parameters()
  {
    // Update model
    cxt_.model_.fluid_density_ = cxt_.fluid_density_;
  }

public:
  DepthNode()
  : Node{"depth_node"}
  {
    (void) baro_sub_;

    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(DEPTH_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), DEPTH_NODE_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    DEPTH_NODE_ALL_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), DEPTH_NODE_ALL_PARAMS)

    // Subscribe
    baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
      "barometer", QUEUE_SIZE, [this](const orca_msgs::msg::Barometer::SharedPtr msg) -> void
      {this->baro_cb_.call(msg);});

    // Advertise
    depth_pub_ = create_publisher<orca_msgs::msg::Depth>("depth", QUEUE_SIZE);

    RCLCPP_INFO(get_logger(), "depth_node ready");
  }

  ~DepthNode() override = default;
};

}  // namespace orca_filter

//=============================================================================
// Main
//=============================================================================

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_filter::DepthNode>();

  // Set logger level
  auto result =
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  (void) result;

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
