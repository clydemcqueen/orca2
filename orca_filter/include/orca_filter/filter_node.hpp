#ifndef ORCA_FILTER_FILTER_NODE_HPP
#define ORCA_FILTER_FILTER_NODE_HPP

#include "orca_description/parser.hpp"
#include "orca_filter/filter_context.hpp"
#include "orca_filter/filter_base.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_msgs/msg/fiducial_pose_stamped2.hpp"
#include "orca_shared/monotonic.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;

namespace orca_filter
{

  class FilterNode : public rclcpp::Node
  {
    // Parameters
    FilterContext cxt_;

    // Timeouts, set by parameters
    rclcpp::Duration open_water_timeout_{0};
    rclcpp::Duration outlier_timeout_{0};

    // Parsed URDF
    orca_description::Parser parser_;

    // Filter state
    bool good_pose_{false};
    std::shared_ptr<FilterBase> filter_;
    rclcpp::Time last_fp_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_fp_inlier_stamp_{0, 0, RCL_ROS_TIME};
    mw::Observations observations_{};

    // Control state
    // double estimated_yaw_{};                      // Yaw used to rotate thruster commands into the world frame
    // orca::Acceleration u_bar_{};                  // Last control, used for filter predict step

    rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped2>::SharedPtr filtered_odom_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

    rclcpp::Subscription<orca_msgs::msg::Depth>::SharedPtr depth_sub_;
    rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
    rclcpp::Subscription<orca_msgs::msg::FiducialPoseStamped2>::SharedPtr fcam_sub_;

    // Validate parameters
    void validate_parameters();

    // Using observations_, set the state and create the appropriate filter
    void create_filter();

    // Callbacks
    void depth_callback(orca_msgs::msg::Depth::SharedPtr msg, bool first);

    void control_callback(orca_msgs::msg::Control::SharedPtr msg, bool first);

    void fcam_callback(orca_msgs::msg::FiducialPoseStamped2::SharedPtr msg, bool first);

    // Callback wrappers
    monotonic::Monotonic<FilterNode *, const orca_msgs::msg::Depth::SharedPtr>
      depth_cb_{this, &FilterNode::depth_callback};
    monotonic::Monotonic<FilterNode *, const orca_msgs::msg::Control::SharedPtr>
      control_cb_{this, &FilterNode::control_callback};
    monotonic::Monotonic<FilterNode *, const orca_msgs::msg::FiducialPoseStamped2::SharedPtr>
      fcam_cb_{this, &FilterNode::fcam_callback};

    // Process a camera pose
    void process_pose(const orca_msgs::msg::FiducialPoseStamped2::SharedPtr &msg,
                      const tf2::Transform &t_sensor_base, const std::string &frame_id);

  public:
    explicit FilterNode();

    ~FilterNode() override = default;
  };

} // namespace orca_filter

#endif // ORCA_FILTER_FILTER_NODE_HPP
