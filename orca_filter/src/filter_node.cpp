#include "orca_filter/filter_node.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_shared/util.hpp"

using namespace orca;

namespace orca_filter
{

  //=============================================================================
  // FilterNode
  //=============================================================================

  FilterNode::FilterNode() : Node{"filter_node"}
  {
    // Suppress IDE warnings
    (void) depth_sub_;
    (void) control_sub_;
    (void) fcam_sub_;
    (void) lcam_sub_;
    (void) rcam_sub_;

    // Get parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(FILTER_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), FILTER_NODE_ALL_PARAMS, validate_parameters)

    // Publication(s)
    filtered_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 1);

    RCLCPP_INFO(get_logger(), "filter_node ready");
  }

  void FilterNode::validate_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_DEBUG, get_logger(), cxt_, n, t, d)
    FILTER_NODE_ALL_PARAMS

    // Set up additional publications and subscriptions
    if (cxt_.filter_baro_) {
      depth_sub_ = create_subscription<orca_msgs::msg::Depth>(
        "depth", 1, [this](const orca_msgs::msg::Depth::SharedPtr msg) -> void
        { this->depth_cb_.call(msg); });
    } else {
      depth_sub_ = nullptr;
    }

    if (cxt_.filter_fcam_) {
      fcam_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "fcam_f_map", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void
        { this->fcam_cb_.call(msg); });
      fcam_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("raw_fcam_f_base", 1);

    } else {
      fcam_sub_ = nullptr;
      fcam_pub_ = nullptr;
    }

    if (cxt_.filter_lcam_) {
      lcam_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "lcam_f_map", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void
        { this->lcam_cb_.call(msg); });
      lcam_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("raw_lcam_f_base", 1);

    } else {
      lcam_sub_ = nullptr;
      lcam_pub_ = nullptr;
    }

    if (cxt_.filter_rcam_) {
      rcam_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "rcam_f_map", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void
        { this->rcam_cb_.call(msg); });
      rcam_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("raw_rcam_f_base", 1);
    } else {
      rcam_sub_ = nullptr;
      rcam_pub_ = nullptr;
    }

    if (cxt_.publish_tf_ || cxt_.publish_measurement_tf_) {
      tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);
    } else {
      tf_pub_ = nullptr;
    }

    // Update model from new parameters
    cxt_.model_.fluid_density_ = cxt_.param_fluid_density_;

    create_filter();

    parse_urdf();
  }

  void FilterNode::create_filter()
  {
    if (receiving_poses_) {
      if (cxt_.four_dof_) {
        RCLCPP_INFO(get_logger(), "4dof pose filter");
        filter_ = std::make_shared<FourFilter>(get_logger(), cxt_);
      } else {
        RCLCPP_INFO(get_logger(), "6dof pose filter");
        filter_ = std::make_shared<PoseFilter>(get_logger(), cxt_);
      }
    } else {
      RCLCPP_INFO(get_logger(), "depth filter");
      filter_ = std::make_shared<DepthFilter>(get_logger(), cxt_);
    }
  }

  void FilterNode::parse_urdf()
  {
    urdf::Model model;
    if (model.initFile(cxt_.urdf_file_)) {
      get_joint(model, cxt_.urdf_forward_camera_joint_, t_fcam_base_);
      get_joint(model, cxt_.urdf_left_camera_joint_, t_lcam_base_);
      get_joint(model, cxt_.urdf_right_camera_joint_, t_rcam_base_);
    } else {
      RCLCPP_ERROR(get_logger(), "failed to parse %s", cxt_.urdf_file_.c_str());
    }
  }

  void FilterNode::get_joint(const urdf::Model &model, const std::string &name, tf2::Transform &t)
  {
    auto joint = model.getJoint(name);
    if (joint) {
      if (joint->parent_link_name != cxt_.frame_id_base_link_) {
        RCLCPP_ERROR(get_logger(), "joint %s expected parent %s, but found %s", name.c_str(),
                     cxt_.frame_id_base_link_.c_str(), joint->parent_link_name.c_str());
      }

      auto pose = joint->parent_to_joint_origin_transform;

      tf2::Transform t2 = tf2::Transform{
        tf2::Quaternion{pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w},
        tf2::Vector3{pose.position.x, pose.position.y, pose.position.z}};

      RCLCPP_DEBUG(get_logger(), "%s: parent(%s), child(%s), %s", name.c_str(),
                   joint->parent_link_name.c_str(), joint->child_link_name.c_str(), to_str_rpy(t2).c_str());

      // Invert
      t = t2.inverse();
      RCLCPP_DEBUG(get_logger(), "inverted %s: parent(%s), child(%s), %s", name.c_str(),
                   joint->parent_link_name.c_str(), joint->child_link_name.c_str(), to_str_rpy(t).c_str());

    } else {
      RCLCPP_ERROR(get_logger(), "joint %s missing", name.c_str());
    }
  }

  // New depth reading
  void FilterNode::depth_callback(const orca_msgs::msg::Depth::SharedPtr msg, bool first)
  {
    if (cxt_.filter_baro_) {

      // If we've stopped receiving poses switch to a depth filter
      if (receiving_poses_) {
        rclcpp::Time stamp{msg->header.stamp};
        if (valid_stamp(last_pose_received_) && stamp - last_pose_received_ > OPEN_WATER_TIMEOUT) {
          RCLCPP_INFO(get_logger(), "running in open water");
          receiving_poses_ = false;
          create_filter();
        }
      }

      nav_msgs::msg::Odometry filtered_odom;
      filtered_odom.header.frame_id = cxt_.frame_id_map_;
      filtered_odom.child_frame_id = cxt_.frame_id_base_link_;

      // Run this message through the filter and publish odometry
      if (filter_->process_message(*msg, u_bar_, filtered_odom)) {
        // Save estimated yaw, used to rotate control messages
        estimated_yaw_ = get_yaw(filtered_odom.pose.pose.orientation);

        publish_odom(filtered_odom);
      }
    }

  }

  void FilterNode::control_callback(const orca_msgs::msg::Control::SharedPtr msg, bool first)
  {
    // TODO must add force parameters to orca_filter to handle u_bar input
#if 0
    Efforts e;
    e.from_msg(msg->efforts);
    e.to_acceleration(estimated_yaw_, u_bar_);
#endif
  }

  void FilterNode::fcam_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first)
  {
    if (cxt_.filter_fcam_) {
      process_pose(msg, t_fcam_base_, "fcam_measurement", fcam_pub_);
    }
  }

  void FilterNode::lcam_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first)
  {
    if (cxt_.filter_lcam_) {
      process_pose(msg, t_lcam_base_, "lcam_measurement", lcam_pub_);
    }
  }

  void FilterNode::rcam_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first)
  {
    if (cxt_.filter_rcam_) {
      process_pose(msg, t_rcam_base_, "rcam_measurement", rcam_pub_);
    }
  }

  void FilterNode::process_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &sensor_f_map,
                                const tf2::Transform &t_sensor_base, const std::string &frame_id,
                                const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr &pose_pub)
  {
    // If we're using a depth filter, switch to a pose filter
    if (!receiving_poses_) {
      RCLCPP_INFO(get_logger(), "found marker(s)");
      receiving_poses_ = true;
      create_filter();
    }

    last_pose_received_ = sensor_f_map->header.stamp;

    // Convert pose to transform
    tf2::Transform t_map_sensor;
    tf2::fromMsg(sensor_f_map->pose.pose, t_map_sensor);

    // Multiply transforms to get t_map_base
    tf2::Transform t_map_base = t_map_sensor * t_sensor_base;

    // Convert transform back to pose
    geometry_msgs::msg::PoseWithCovarianceStamped base_f_map;
    base_f_map.header = sensor_f_map->header;
    toMsg(t_map_base, base_f_map.pose.pose);
    base_f_map.pose.covariance = sensor_f_map->pose.covariance;  // TODO rotate covariance

    // Publish unfiltered pose
    if (pose_pub->get_subscription_count() > 0) {
      pose_pub->publish(base_f_map);
    }

    // Publish unfiltered tf
    if (cxt_.publish_measurement_tf_ && tf_pub_->get_subscription_count() > 0) {
      geometry_msgs::msg::TransformStamped geo_tf;
      geo_tf.header = base_f_map.header;
      geo_tf.child_frame_id = frame_id;
      geo_tf.transform = toMsg(t_map_base);

      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.emplace_back(geo_tf);

      tf_pub_->publish(tf_message);
    }

    // If we're receiving poses but not publishing odometry then reset the filter
    rclcpp::Time stamp{sensor_f_map->header.stamp};
    if (valid_stamp(last_pose_inlier_) && stamp - last_pose_inlier_ > OUTLIER_TIMEOUT) {
      RCLCPP_WARN(get_logger(), "reset filter");
      filter_->reset(base_f_map.pose.pose);
      last_pose_inlier_ = stamp;
    }

    nav_msgs::msg::Odometry filtered_odom;
    filtered_odom.header.frame_id = cxt_.frame_id_map_;
    filtered_odom.child_frame_id = cxt_.frame_id_base_link_;

    if (filter_->process_message(base_f_map, u_bar_, filtered_odom)) {
      // Save estimated yaw, used to rotate control messages
      estimated_yaw_ = get_yaw(filtered_odom.pose.pose.orientation);

      publish_odom(filtered_odom);

      last_pose_inlier_ = stamp;
    }
  }

  void FilterNode::publish_odom(nav_msgs::msg::Odometry &odom)
  {
    // Publish odometry
    if (filtered_odom_pub_->get_subscription_count() > 0) {
      filtered_odom_pub_->publish(odom);
    }

    // Publish filtered tf
    if (cxt_.publish_tf_ && tf_pub_->get_subscription_count() > 0) {
      geometry_msgs::msg::TransformStamped geo_tf;
      geo_tf.header = odom.header;
      geo_tf.child_frame_id = cxt_.frame_id_base_link_;

      // geometry_msgs::msg::Pose -> tf2::Transform -> geometry_msgs::msg::Transform
      tf2::Transform t_map_base;
      fromMsg(odom.pose.pose, t_map_base);
      geo_tf.transform = toMsg(t_map_base);

      // One transform in this tf message
      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.emplace_back(geo_tf);

      tf_pub_->publish(tf_message);
    }
  }

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
  auto node = std::make_shared<orca_filter::FilterNode>();

  // Set logger level
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
