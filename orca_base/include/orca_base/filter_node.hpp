#ifndef ORCA_BASE_FILTER_NODE_HPP
#define ORCA_BASE_FILTER_NODE_HPP

#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/depth.hpp"

#include "orca_base/filter_context.hpp"
#include "orca_base/filter.hpp"
#include "orca_base/monotonic.hpp"

using namespace std::chrono_literals;

namespace orca_base
{

  class FilterNode : public rclcpp::Node
  {
  private:

    const std::chrono::milliseconds SPIN_PERIOD{50ms};   // Publish messages at 20Hz

    // Parameters
    FilterContext cxt_;

    // UKF state
    std::shared_ptr<Filter> filter_;
    bool filter_valid_{true};                     // True if the filter is valid

    // Control state
    Acceleration u_bar_;                          // Last control, used for filter predict step

    // Barometer state
    bool z_valid_{false};                         // True if z_ is valid
    double z_offset_{};                           // Z offset, see baro_callback()
    double z_{};                                  // Z from barometer

    rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
    rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr spin_timer_;

    // Validate parameters
    void validate_parameters();

    // Callbacks
    void baro_callback(orca_msgs::msg::Barometer::SharedPtr msg);

    void control_callback(orca_msgs::msg::Control::SharedPtr msg);

    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);

    // Callback wrappers
    Valid<FilterNode *, const orca_msgs::msg::Barometer::SharedPtr> baro_cb_{this, &FilterNode::baro_callback};
    Valid<FilterNode *, const orca_msgs::msg::Control::SharedPtr> control_cb_{this, &FilterNode::control_callback};
    Valid<FilterNode *, const nav_msgs::msg::Odometry::SharedPtr> odom_cb_{this, &FilterNode::odom_callback};

    // Publications
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
    rclcpp::Publisher<orca_msgs::msg::Depth>::SharedPtr depth_pub_;

  public:
    explicit FilterNode();

    ~FilterNode() override = default;

    void spin_once();
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_NODE_HPP
