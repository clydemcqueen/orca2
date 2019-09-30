#include "orca_base/filter_node.hpp"
#include "orca_base/pwm.hpp"

namespace orca_base
{

  //=============================================================================
  // FilterNode
  //=============================================================================

  FilterNode::FilterNode() : Node{"filter_node"}
  {
    // Suppress IDE warnings
    (void) baro_sub_;
    (void) control_sub_;
    (void) odom_sub_;
    (void) spin_timer_;

    // Get parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(FILTER_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), FILTER_NODE_ALL_PARAMS, validate_parameters)

    // Odom filter
    filter_ = std::make_shared<Filter>(get_logger(), cxt_);

    // Publications
    filtered_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("filtered_odom", 1);
    depth_pub_ = create_publisher<orca_msgs::msg::Depth>("depth", 1);

    // Monotonic subscriptions
    baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
      "barometer", 1, [this](const orca_msgs::msg::Barometer::SharedPtr msg) -> void
      { this->baro_cb_.call(msg); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "fiducial_odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void
      { this->odom_cb_.call(msg); });

    // Loop will run at ~constant wall speed, switch to ros_timer when it exists
    spin_timer_ = create_wall_timer(SPIN_PERIOD, std::bind(&FilterNode::spin_once, this));

    RCLCPP_INFO(get_logger(), "filter_node ready");
  }

  void FilterNode::validate_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    FILTER_NODE_ALL_PARAMS

    // Update model from new parameters
    cxt_.model_.fluid_density_ = cxt_.model_fluid_density_;
  }

  // New barometer reading
  void FilterNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg)
  {
    // Calc depth from pressure, this assumes constant air pressure
    double z = cxt_.model_.pressure_to_z(msg->pressure);

    // Three ways to initialize the barometer:
    // cxt_.baro_init_ == 0:   Orca is in the air, so the first z reading is just air pressure
    // cxt_.baro_init_ == 1:   Orca is floating at the surface in the water, the barometer is submerged ~5cm
    // cxt_.baro_init_ == 2:   Wait for good odometry from fiducial_vlam and initialize barometer from the map

    // TODO pull these from the URDF
    static const double z_top_to_baro_link = -0.05;
    static const double z_baro_link_to_base_link = -0.085;

    if (!z_valid_ && cxt_.baro_init_ == 0) {
      z_offset_ = -z + z_top_to_baro_link + z_baro_link_to_base_link;
      z_valid_ = true;
      RCLCPP_INFO(get_logger(), "barometer init mode 0 (in air): adjustment %g", z_offset_);
    } else if (!z_valid_ && cxt_.baro_init_ == 1) {
      z_offset_ = -z + z_baro_link_to_base_link;
      z_valid_ = true;
      RCLCPP_INFO(get_logger(), "barometer init mode 1 (in water): adjustment %g", z_offset_);
    }

    if (z_valid_) {
      // Adjust reading
      z_ = z + z_offset_;

      orca_msgs::msg::Depth depth_msg;
      depth_msg.header = msg->header;
      depth_msg.z = z_;
      depth_msg.z_variance = Model::DEPTH_STDDEV * Model::DEPTH_STDDEV;

      // Publish depth, useful for diagnostics
      if (depth_pub_->get_subscription_count() > 0) {
        depth_pub_->publish(depth_msg);
      }

      if (cxt_.filter_baro_) {
        filter_->queue_depth(depth_msg);
      }
    }
  }

  void FilterNode::control_callback(const orca_msgs::msg::Control::SharedPtr msg)
  {
    double yaw{};  // TODO
    Efforts e;
    e.from_msg(msg->efforts);
    e.to_acceleration(u_bar_, yaw);
  }

  void FilterNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    filter_->queue_odom(*msg);
  }

  void FilterNode::spin_once()
  {
    if (!filter_valid_) {
      return;
    }

    auto spin_time = now();
    if (spin_time.nanoseconds() <= 0) {
      return;
    }

    // Filter the odometry, passing in the previous acceleration as the control
    nav_msgs::msg::Odometry filtered_odom;
    filter_valid_ = filter_->process(spin_time, u_bar_, filtered_odom);
    if (!filter_valid_) {
      RCLCPP_ERROR(get_logger(), "filter is invalid");
      return;
    }

    // Publish filtered odometry
    if (filtered_odom_pub_->get_subscription_count() > 0) {
      filtered_odom_pub_->publish(filtered_odom);
    }
  }

} // namespace orca_base

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
  auto node = std::make_shared<orca_base::FilterNode>();

  // Set logger level
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
