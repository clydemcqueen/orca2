#ifndef ORCA_FILTER_FILTER_NODE_HPP
#define ORCA_FILTER_FILTER_NODE_HPP

#include "urdf/model.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/depth.hpp"

#include "orca_shared/monotonic.hpp"

#include "orca_filter/filter_context.hpp"
#include "orca_filter/filter_base.hpp"

using namespace std::chrono_literals;

namespace orca_filter
{

  class FilterNode : public rclcpp::Node
  {
  private:

    // FilterNode runs one of two filters:
    // -- a pose filter takes all sensor input and produces a full pose
    // -- a depth filter takes barometric data and produces a filtered z value
    //
    // When poses are available (near a marker) the pose filter is used.
    // When poses are not available (running through open water) the depth filter is used.
    //
    // Both filters publish odometry. The depth filter publishes very high (>1e4) covariance values for most dimensions.

    bool receiving_poses_{false};
    std::shared_ptr<FilterBase> filter_;
    rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_pose_inlier_stamp_{0, 0, RCL_ROS_TIME};

    // Timeouts, set by parameters
    rclcpp::Duration open_water_timeout_{0};
    rclcpp::Duration outlier_timeout_{0};

    // Parameters
    FilterContext cxt_;

    // Control state
    double estimated_yaw_{};                      // Yaw used to rotate thruster commands into the world frame
    orca::Acceleration u_bar_{};                  // Last control, used for filter predict step

    // Transform base_f_sensor_frame for camera sensors
    tf2::Transform t_fcam_base_{};
    tf2::Transform t_lcam_base_{};
    tf2::Transform t_rcam_base_{};

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fcam_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lcam_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rcam_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

    rclcpp::Subscription<orca_msgs::msg::Depth>::SharedPtr depth_sub_;
    rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fcam_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lcam_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rcam_sub_;

    // Validate parameters
    void validate_parameters();

    // Create the filter
    void create_filter();

    // Parse urdf
    void parse_urdf();

    void get_joint(const urdf::Model &model, const std::string &name, tf2::Transform &t);

    // Callbacks
    void depth_callback(orca_msgs::msg::Depth::SharedPtr msg, bool first);

    void control_callback(orca_msgs::msg::Control::SharedPtr msg, bool first);

    void fcam_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first);

    void lcam_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first);

    void rcam_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first);

    // Callback wrappers
    monotonic::Monotonic<FilterNode *, const orca_msgs::msg::Depth::SharedPtr>
      depth_cb_{this, &FilterNode::depth_callback};
    monotonic::Monotonic<FilterNode *, const orca_msgs::msg::Control::SharedPtr>
      control_cb_{this, &FilterNode::control_callback};
    monotonic::Monotonic<FilterNode *, const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>
      fcam_cb_{this, &FilterNode::fcam_callback};
    monotonic::Monotonic<FilterNode *, const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>
      lcam_cb_{this, &FilterNode::lcam_callback};
    monotonic::Monotonic<FilterNode *, const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>
      rcam_cb_{this, &FilterNode::rcam_callback};

    // Process a camera pose
    void process_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &sensor_f_map,
                      const tf2::Transform &t_sensor_base, const std::string &frame_id,
                      const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr &pose_pub);

  public:
    explicit FilterNode();

    ~FilterNode() override = default;
  };

} // namespace orca_filter

#endif // ORCA_FILTER_FILTER_NODE_HPP
