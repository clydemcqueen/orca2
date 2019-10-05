#include "orca_base/filter_node.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

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
    (void) fcam_sub_;
    (void) lcam_sub_;
    (void) rcam_sub_;
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
    fcam_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("fcam_f_base", 1);
    lcam_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("lcam_f_base", 1);
    rcam_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("rcam_f_base", 1);
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);

    // Monotonic subscriptions
    baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
      "barometer", 1, [this](const orca_msgs::msg::Barometer::SharedPtr msg) -> void
      { this->baro_cb_.call(msg); });
    fcam_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "fcam_f_map", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void
      { this->fcam_cb_.call(msg); });
    lcam_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "lcam_f_map", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void
      { this->lcam_cb_.call(msg); });
    rcam_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "rcam_f_map", 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void
      { this->rcam_cb_.call(msg); });

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
    cxt_.model_.fluid_density_ = cxt_.param_fluid_density_;

    // Parse URDF
    parse_urdf();
  }

  void FilterNode::parse_urdf()
  {
    urdf::Model model;
    if (model.initFile(cxt_.urdf_file_)) {
      get_joint(model, cxt_.urdf_barometer_joint_, t_baro_base_);
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

      RCLCPP_INFO(get_logger(), "%s: parent(%s), child(%s), %s", name.c_str(),
                  joint->parent_link_name.c_str(), joint->child_link_name.c_str(), to_str_rpy(t2).c_str());

      // Invert
      t = t2.inverse();
      RCLCPP_INFO(get_logger(), "inverted %s: parent(%s), child(%s), %s", name.c_str(),
                  joint->parent_link_name.c_str(), joint->child_link_name.c_str(), to_str_rpy(t).c_str());

    } else {
      RCLCPP_ERROR(get_logger(), "joint %s missing", name.c_str());
    }
  }

  // New barometer reading
  void FilterNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg, bool first)
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
      depth_msg.z_variance = Model::DEPTH_STDDEV * Model::DEPTH_STDDEV * 10; // Boost measurement uncertainty

      // Publish depth, useful for diagnostics
      if (depth_pub_->get_subscription_count() > 0) {
        depth_pub_->publish(depth_msg);
      }

      if (cxt_.filter_baro_) {
        filter_->queue_depth(depth_msg);
      }
    }
  }

  void FilterNode::control_callback(const orca_msgs::msg::Control::SharedPtr msg, bool first)
  {
    double yaw{};  // TODO
    Efforts e;
    e.from_msg(msg->efforts);
    e.to_acceleration(u_bar_, yaw);
  }

  void FilterNode::fcam_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first)
  {
    queue_pose(msg, t_fcam_base_, "fcam_measurement", fcam_pub_);
  }

  void FilterNode::lcam_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first)
  {
    queue_pose(msg, t_lcam_base_, "lcam_measurement", lcam_pub_);
  }

  void FilterNode::rcam_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, bool first)
  {
    queue_pose(msg, t_rcam_base_, "rcam_measurement", rcam_pub_);
  }

  void FilterNode::queue_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &sensor_f_map,
                              const tf2::Transform &t_sensor_base, const std::string &frame_id,
                              const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr &pose_pub)
  {
    // Convert pose to transform
    tf2::Transform t_map_sensor;
    tf2::fromMsg(sensor_f_map->pose.pose, t_map_sensor);

    // Apply transform to get t_map_base
    tf2::Transform t_map_base = t_map_sensor * t_sensor_base;

    // Convert transform back to pose
    geometry_msgs::msg::PoseWithCovarianceStamped base_f_map;
    base_f_map.header = sensor_f_map->header;
    toMsg(t_map_base, base_f_map.pose.pose);
    base_f_map.pose.covariance = sensor_f_map->pose.covariance;  // TODO rotate covariance

    // Optionally publish pose for diagnostics
    if (pose_pub->get_subscription_count() > 0) {
      pose_pub->publish(base_f_map);
    }

    // Optionally publish tf for diagnostics
    if (cxt_.publish_measurement_tf_ && tf_pub_->get_subscription_count() > 0) {
      geometry_msgs::msg::TransformStamped geo_tf;
      geo_tf.header = base_f_map.header;
      geo_tf.child_frame_id = frame_id;
      geo_tf.transform = toMsg(t_map_base);

      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.emplace_back(geo_tf);

      tf_pub_->publish(tf_message);
    }

    // Queue
    filter_->queue_pose(base_f_map);
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
    filtered_odom.header.frame_id = cxt_.frame_id_map_;
    filtered_odom.child_frame_id = cxt_.frame_id_base_link_;

    // Publish filtered odometry
    if (filtered_odom_pub_->get_subscription_count() > 0) {
      filtered_odom_pub_->publish(filtered_odom);
    }

    // Publish tf
    if (cxt_.publish_filtered_tf_ && tf_pub_->get_subscription_count() > 0) {
      geometry_msgs::msg::TransformStamped geo_tf;
      geo_tf.header = filtered_odom.header;
      geo_tf.child_frame_id = cxt_.frame_id_base_link_;

      // geometry_msgs::msg::Pose -> tf2::Transform -> geometry_msgs::msg::Transform
      tf2::Transform t_map_base;
      fromMsg(filtered_odom.pose.pose, t_map_base);
      geo_tf.transform = toMsg(t_map_base);

      // One transform in this tf message
      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.emplace_back(geo_tf);

      tf_pub_->publish(tf_message);
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
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
