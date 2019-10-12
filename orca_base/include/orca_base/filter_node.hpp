#ifndef ORCA_BASE_FILTER_NODE_HPP
#define ORCA_BASE_FILTER_NODE_HPP

#include "urdf/model.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/depth.hpp"

#include "orca_base/filter_context.hpp"
#include "orca_base/filter_base.hpp"
#include "orca_base/monotonic.hpp"

using namespace std::chrono_literals;

namespace orca_base
{

  class FilterNode : public rclcpp::Node
  {
  private:

    const rclcpp::Duration FILTER_TIMEOUT{RCL_S_TO_NS(1)};  // Reset filter if it borks

    // Parameters
    FilterContext cxt_;

    // UKF state
    std::shared_ptr<FilterBase> odom_filter_;
    rclcpp::Time publish_time_{0, 0, RCL_ROS_TIME};

    // Control state
    Acceleration u_bar_;                          // Last control, used for filter predict step

    // Barometer state
    bool z_valid_{false};                         // True if z_ is valid
    double z_offset_{};                           // Z offset, see baro_callback()
    double z_{};                                  // Z from barometer

    // Transform base_f_sensor_frame for all sensors
    tf2::Transform t_baro_base_{};
    tf2::Transform t_fcam_base_{};
    tf2::Transform t_lcam_base_{};
    tf2::Transform t_rcam_base_{};

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
    rclcpp::Publisher<orca_msgs::msg::Depth>::SharedPtr depth_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fcam_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lcam_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rcam_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

    rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
    rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fcam_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lcam_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rcam_sub_;

    // Validate parameters
    void validate_parameters();

    // Parse urdf
    void parse_urdf();

    void get_joint(const urdf::Model &model, const std::string &name, tf2::Transform &t);

    // Callbacks
    void baro_callback(orca_msgs::msg::Barometer::SharedPtr msg, bool first);

    void control_callback(orca_msgs::msg::Control::SharedPtr msg, bool first);

    void fcam_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first);

    void lcam_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first);

    void rcam_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first);

    // Callback wrappers
    Monotonic<FilterNode *, const orca_msgs::msg::Barometer::SharedPtr> baro_cb_{this, &FilterNode::baro_callback};
    Monotonic<FilterNode *, const orca_msgs::msg::Control::SharedPtr> control_cb_{this, &FilterNode::control_callback};
    Monotonic<FilterNode *,
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> fcam_cb_{this, &FilterNode::fcam_callback};
    Monotonic<FilterNode *,
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> lcam_cb_{this, &FilterNode::lcam_callback};
    Monotonic<FilterNode *,
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> rcam_cb_{this, &FilterNode::rcam_callback};

    // Process a camera pose
    void process_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &sensor_f_map,
                      const tf2::Transform &t_sensor_base, const std::string &frame_id,
                      const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr &pose_pub);

    // Filter a message and publish odometry
    template<typename T>
    void filter_odom(const T &msg);

  public:
    explicit FilterNode();

    ~FilterNode() override = default;
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_NODE_HPP
