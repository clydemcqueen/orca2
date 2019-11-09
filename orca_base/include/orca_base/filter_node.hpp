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
    rclcpp::Time last_pose_received_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_pose_inlier_{0, 0, RCL_ROS_TIME};

    // Change modes if poses are missing, or available, for 0.3s (~9 poses)
    const rclcpp::Duration OPEN_WATER_TIMEOUT{RCL_MS_TO_NS(300)};

    // Reset the filter if poses are consistently rejected as outliers for 0.3s (~9 poses)
    const rclcpp::Duration OUTLIER_TIMEOUT{RCL_MS_TO_NS(300)};

    // Parameters
    FilterContext cxt_;

    // Control state
    double estimated_yaw_{};                      // Yaw used to rotate thruster commands into the world frame
    Acceleration u_bar_{};                        // Last control, used for filter predict step

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

    // Create the filter
    void create_filter();

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

    // Publish odometry
    void publish_odom(nav_msgs::msg::Odometry &odom);

  public:
    explicit FilterNode();

    ~FilterNode() override = default;
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_NODE_HPP
