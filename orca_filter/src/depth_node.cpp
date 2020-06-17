#include "rclcpp/node.hpp"

#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_shared/baro.hpp"
#include "orca_shared/monotonic.hpp"
#include "ros2_shared/context_macros.hpp"

namespace orca_filter
{

  //=============================================================================
  // Parameter(s)
  //=============================================================================

#define DEPTH_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(fluid_density, double, 997)          /* kg/m^3, 997 for freshwater, 1029 for seawater  */ \
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
      depth_msg.header.frame_id = "map"; // TODO parameter

      // Convert pressure at baro_link to depth at base_link
      depth_msg.z = barometer_.pressure_to_base_link_z(cxt_.model_, baro_msg->pressure);

      // Boost measurement uncertainty TODO parameter
      depth_msg.z_variance = orca::Model::DEPTH_STDDEV * orca::Model::DEPTH_STDDEV * 10;

      depth_pub_->publish(depth_msg);
    }

    // Validate parameters
    void validate_parameters()
    {
      // Update model
      cxt_.model_.fluid_density_ = cxt_.fluid_density_;
    }

  public:

    explicit DepthNode() : Node{"depth_node"}
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
        { this->baro_cb_.call(msg); });

      // Advertise
      depth_pub_ = create_publisher<orca_msgs::msg::Depth>("depth", QUEUE_SIZE);

      RCLCPP_INFO(get_logger(), "depth_node ready");
    }

    ~DepthNode() override = default;
  };

} // namespace orca_filter

//=============================================================================
// Main
//=============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_filter::DepthNode>();

  // Set logger level
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  (void) result;

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}