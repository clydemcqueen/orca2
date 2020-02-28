#include "orca_base/auv_node.hpp"

#include <iomanip>
#include <utility>

#include "cv_bridge/cv_bridge.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_shared/pwm.hpp"

#include "orca_base/thrusters.hpp"

using namespace orca;

namespace orca_base
{

  //=============================================================================
  // Utilities
  //=============================================================================

  void draw_text(cv::Mat &image, const std::string &s, const cv::Point2d &p, cv::Scalar color)
  {
    putText(image, s.c_str(), p, cv::FONT_HERSHEY_PLAIN, 1, std::move(color));
  }

  void draw_observations(cv::Mat &image, const std::vector<Observation> &observations, const cv::Scalar &color)
  {
    for (const auto &obs : observations) {
      // Draw marker name
      cv::Point2d p = obs.c3;
      p.y += 30;
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2);

      ss << "m" << obs.id << " d=" << obs.distance;
      draw_text(image, ss.str(), p, color);

      // Draw bounding box
      cv::line(image, obs.c0, obs.c1, color);
      cv::line(image, obs.c1, obs.c2, color);
      cv::line(image, obs.c2, obs.c3, color);
      cv::line(image, obs.c3, obs.c0, color);
    }
  }

  //=============================================================================
  // AUVNode
  //=============================================================================

  AUVNode::AUVNode() : Node{"auv_node"}, map_{get_logger(), cxt_}
  {
    // Suppress IDE warnings
    (void) baro_sub_;
    (void) battery_sub_;
    (void) fcam_image_sub_;
    (void) fcam_info_sub_;
    (void) goal_sub_;
    (void) leak_sub_;
    (void) map_sub_;
    (void) obs_sub_;
    (void) spin_timer_;
    (void) mission_server_;

    // Get parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(AUV_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), AUV_NODE_ALL_PARAMS, validate_parameters)

    // Publications
    control_pub_ = create_publisher<orca_msgs::msg::Control>("control", 1);
    fcam_predicted_obs_pub_ = create_publisher<sensor_msgs::msg::Image>("fcam_predicted_obs", 1);
    esimated_path_pub_ = create_publisher<nav_msgs::msg::Path>("filtered_path", 1);
    planned_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("planned_pose", 1);
    target_path_pub_ = create_publisher<nav_msgs::msg::Path>("target_path", 1);
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);

    // Camera info may be published with a different QoS
    auto camera_info_qos = rclcpp::QoS{rclcpp::SensorDataQoS()};

    // Monotonic subscriptions
    baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
      "barometer", 1, [this](const orca_msgs::msg::Barometer::SharedPtr msg) -> void
      { this->baro_cb_.call(msg); });
    fcam_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "fcam_image", 1, [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
      { this->fcam_image_cb_.call(msg); });
    fcam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "fcam_info", camera_info_qos, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) -> void
      { this->fcam_info_cb_.call(msg); });
    map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
      "fiducial_map", 1, [this](const fiducial_vlam_msgs::msg::Map::SharedPtr msg) -> void
      { this->map_cb_.call(msg); });

    using namespace std::placeholders;

    // Sync subscriptions
    fiducial_sync_.reset(new FiducialSync(FiducialPolicy(1), obs_sub_, fcam_pose_sub_));
    fiducial_sync_->registerCallback(std::bind(&AUVNode::fiducial_callback, this, _1, _2));
    obs_sub_.subscribe(this, "fiducial_observations"); // Uses default qos
    fcam_pose_sub_.subscribe(this, "fcam_f_map"); // Uses default qos

    // Other subscriptions
    auto battery_cb = std::bind(&AUVNode::battery_callback, this, _1);
    battery_sub_ = create_subscription<orca_msgs::msg::Battery>("battery", 1, battery_cb);
    auto leak_cb = std::bind(&AUVNode::leak_callback, this, _1);
    leak_sub_ = create_subscription<orca_msgs::msg::Leak>("leak", 1, leak_cb);

    // Action server
    mission_server_ = rclcpp_action::create_server<MissionAction>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "mission",
      std::bind(&AUVNode::mission_goal, this, _1, _2),
      std::bind(&AUVNode::mission_cancel, this, _1),
      std::bind(&AUVNode::mission_accepted, this, _1));

    // Parse URDF
    if (!parser_.parse()) {
      RCLCPP_ERROR(get_logger(), "can't parse URDF %s", orca_description::filename);
    } else {
      std::cout << "baro " << to_str_rpy(parser_.t_baro_base) << std::endl;
      std::cout << "fcam " << to_str_rpy(parser_.t_fcam_base) << std::endl;
      std::cout << "lcam " << to_str_rpy(parser_.t_lcam_base) << std::endl;
      std::cout << "rcam " << to_str_rpy(parser_.t_rcam_base) << std::endl;
    }

    RCLCPP_INFO(get_logger(), "auv_node ready");
  }

  void AUVNode::validate_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_DEBUG, get_logger(), cxt_, n, t, d)
    AUV_NODE_ALL_PARAMS

    // Update model from new parameters
    cxt_.model_.fluid_density_ = cxt_.fluid_density_;
    cxt_.model_.drag_coef_f_ = cxt_.drag_coef_f_;
    cxt_.model_.drag_coef_s_ = cxt_.drag_coef_s_;
    cxt_.model_.drag_coef_z_ = cxt_.drag_coef_z_;
    cxt_.model_.drag_coef_tether_ = cxt_.drag_coef_tether_;
    cxt_.model_.drag_partial_const_yaw_ = cxt_.drag_partial_const_yaw_;

    // Update timeouts
    baro_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_baro_ms_)};
    fp_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_fp_ms_)};
    spin_period_ = std::chrono::milliseconds{cxt_.timer_period_ms_};

    // [Re-]start loop
    // Loop will run at ~constant wall speed, switch to ros_timer when it exists
    spin_timer_ = create_wall_timer(spin_period_, std::bind(&AUVNode::spin_once, this));
  }

  bool AUVNode::baro_ok(const rclcpp::Time &t)
  { return barometer_.initialized() && baro_cb_.receiving() && t - baro_cb_.prev() < baro_timeout_; }

  bool AUVNode::fp_ok(const rclcpp::Time &t)
  { return monotonic::valid(estimate_.t) && t - estimate_.t < fp_timeout_; }

  bool AUVNode::cam_info_ok()
  { return fcam_info_cb_.receiving(); }

  bool AUVNode::ready_to_start_mission()
  {
    // -- not currently in a mission
    // -- have a map
    // -- have camera info
    // -- have a good pose or a good observation
    return !mission_ && map_.ok() && cam_info_ok() && fp_ok(now()) &&
           (estimate_.fp.good_pose(cxt_.good_pose_dist_) || estimate_.fp.has_good_observation(cxt_.good_obs_dist_));
  }

  // Timer callback
  void AUVNode::spin_once()
  {
    // Ignore 0
    auto spin_time = now();
    if (spin_time.nanoseconds() <= 0) {
      return;
    }

    // Various timeouts
    if (mission_ && !baro_ok(spin_time)) {
      RCLCPP_ERROR(get_logger(), "lost barometer, abort mission");
      abort_mission(spin_time);
    }

    if (!fp_ok(spin_time)) {
      // We lost observations, nuke estimate_
      estimate_ = FPStamped{};
    }

    // Mission loop is either timer-driven or sensor driven
    // Note: control_msg.odom_lag will always be 0
    if (!cxt_.sensor_loop_ && mission_) {
      auv_advance(now(), spin_period_);
    }
  }

  // New barometer reading
  void AUVNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg)
  {
    base_link_z_ = barometer_.pressure_to_base_link_z(cxt_.model_, msg->pressure);
  }

  // New battery reading
  void AUVNode::battery_callback(const orca_msgs::msg::Battery::SharedPtr msg)
  {
    if (mission_ && msg->low_battery) {
      RCLCPP_ERROR(get_logger(), "low battery (%g volts), abort mission", msg->voltage);
      abort_mission(msg->header.stamp);
    }
  }

  void AUVNode::fcam_image_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (fcam_predicted_obs_pub_->get_subscription_count() > 0) {
      cv_bridge::CvImagePtr marked;
      marked = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      if (mission_) {
        draw_observations(marked->image, mission_->status().pose.fp.observations, CV_RGB(255, 0, 0));
      }
      draw_observations(marked->image, estimate_.fp.observations, CV_RGB(0, 255, 0));
      write_status(marked->image);

      fcam_predicted_obs_pub_->publish(*marked->toImageMsg());
    }
  }

  void AUVNode::fcam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fcam_model_.fromCameraInfo(msg);
  }

  // Leak detector
  void AUVNode::leak_callback(const orca_msgs::msg::Leak::SharedPtr msg)
  {
    if (mission_ && msg->leak_detected) {
      RCLCPP_ERROR(get_logger(), "leak detected, abort mission");
      abort_mission(msg->header.stamp);
    }
  }

  // New map available
  void AUVNode::map_callback(const fiducial_vlam_msgs::msg::Map::SharedPtr msg)
  {
    map_.set_vlam_map(msg);
  }

  // Get a synchronized set of messages from vlam: pose from SolvePnP and raw marker observations
  // Built for the non-filter case TODO also handle the filter case
  void AUVNode::fiducial_callback(
    const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &obs_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &fcam_msg)
  {
    // Ignore until we have a good map and the barometer is initialized TODO filter case
    if (!map_.ok() || !barometer_.initialized()) {
      return;
    }

    rclcpp::Time prev_time = estimate_.t;
    rclcpp::Time curr_time = obs_msg->header.stamp;

    // Ignore invalid timestamps
    if (!monotonic::valid(curr_time)) {
      return;
    }

    // Ignore duplicates, or time going backwards
    if (curr_time <= prev_time) {
      return;
    }

    // First message?
    bool first = !monotonic::valid(prev_time);

    // Convert pose to transform
    tf2::Transform t_map_sensor;
    tf2::fromMsg(fcam_msg->pose.pose, t_map_sensor);

    // Multiply transforms to get t_map_base
    tf2::Transform t_map_base = t_map_sensor * parser_.t_fcam_base;

    // Convert transform back to pose
    geometry_msgs::msg::PoseWithCovarianceStamped base_f_map;
    base_f_map.header = fcam_msg->header;
    toMsg(t_map_base, base_f_map.pose.pose);
    base_f_map.pose.covariance = fcam_msg->pose.covariance;  // TODO rotate covariance

    // Publish tf
    if (cxt_.publish_tf_ && tf_pub_->get_subscription_count() > 0) {
      // Build transform message
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = fcam_msg->header.stamp;
      transform.header.frame_id = cxt_.map_frame_;
      transform.child_frame_id = cxt_.base_frame_;
      transform.transform = toMsg(t_map_base);

      // TF messages can have multiple transforms, we have just 1
      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.emplace_back(transform);

      tf_pub_->publish(tf_message);
    }

    // Save the observations and pose
    // The z value from the barometer is much more accurate TODO handle filter case
    estimate_.from_msgs(*obs_msg, base_f_map, base_link_z_, map_.marker_length(), cxt_.fcam_hfov_, cxt_.fcam_hres_);

    if (cxt_.sensor_loop_ && mission_) {
      // Skip the first message -- the dt will be too large
      if (first) {
        return;
      }

      // Reject large dt
      auto d = curr_time - prev_time;
      if (d.seconds() > 0.5) {
        return;
      }

      // Continue mission
      auv_advance(estimate_.t, d);
    }
  }

  rclcpp_action::GoalResponse AUVNode::mission_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MissionAction::Goal> goal)
  {
    (void) goal;

    using orca_msgs::msg::Control;

    if (ready_to_start_mission()) {
      RCLCPP_INFO(get_logger(), "mission accepted");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_ERROR(get_logger(), "mission rejected"); // TODO would be nice to say why it was rejected
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse AUVNode::mission_cancel(const std::shared_ptr<MissionHandle> goal_handle)
  {
    (void) goal_handle;

    // Always accept a cancellation
    RCLCPP_INFO(get_logger(), "cancel mission");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Start a mission. There are lots of options! Here are some default cases:
  //
  // If !pose_targets, then look to markers for a list of markers.
  // If markers.empty(), then use all markers in the map
  //
  // If pose_targets, then look to poses for a list of poses.
  // If poses.empty(), then try to provide the current pose.
  // This only works if we have a good pose. It's only useful if keep_station is true.
  void AUVNode::mission_accepted(const std::shared_ptr<MissionHandle> goal_handle)
  {
    auto action = goal_handle->get_goal();

    RCLCPP_INFO(get_logger(), "start mission, pose_targets=%d, %d marker(s), %d pose(s), random=%d, repeat=%d, keep=%d",
                action->pose_targets, action->marker_ids.size(), action->poses.size(), action->random, action->repeat,
                action->keep_station);

    // Create a global planner
    ++global_plan_idx_;
    std::shared_ptr<GlobalPlanner> planner;
    if (action->pose_targets) {
      // If poses.empty() then go to the current pose
      if (action->poses.empty() && action->keep_station && estimate_.fp.good_pose(cxt_.good_pose_dist_)) {
        RCLCPP_INFO(get_logger(), "keeping station at current pose");
        std::vector<geometry_msgs::msg::Pose> poses;
        poses.push_back(estimate_.fp.pose.pose.to_msg());
        planner = GlobalPlanner::plan_poses(get_logger(), cxt_, map_, parser_, fcam_model_, poses,
                                            action->random, action->repeat, action->keep_station);
      } else {
        planner = GlobalPlanner::plan_poses(get_logger(), cxt_, map_, parser_, fcam_model_, action->poses,
                                            action->random, action->repeat, action->keep_station);
      }
    } else {
      planner = GlobalPlanner::plan_markers(get_logger(), cxt_, map_, parser_, fcam_model_, action->marker_ids,
                                            action->random, action->repeat, action->keep_station);
    }

    if (!planner) {
      // Failed to build a global plan, abort mission
      auto result = std::make_shared<MissionAction::Result>();
      result->targets_total = 0;
      result->targets_completed = 0;
      goal_handle->abort(result);
      return;
    }

    // Publish global path for rviz
    if (count_subscribers(target_path_pub_->get_topic_name()) > 0) {
      target_path_pub_->publish(planner->global_path());
    }

    // Create mission, passing in the planner
    mission_ = std::make_shared<Mission>(get_logger(), cxt_, goal_handle, planner, estimate_);

    // Init estimated path
    estimated_path_.header.stamp = now();
    estimated_path_.header.frame_id = cxt_.map_frame_;
    estimated_path_.poses.clear();
  }

  void AUVNode::abort_mission(const rclcpp::Time &msg_time)
  {
    if (mission_) {
      mission_->abort();
      mission_ = nullptr;

      // Send the last control message
      publish_control(msg_time, {});
    }
  }

  void AUVNode::auv_advance(const rclcpp::Time &msg_time, const rclcpp::Duration &d)
  {
    // Publish estimated path
    if (estimate_.fp.good_pose(cxt_.good_pose_dist_) && count_subscribers(esimated_path_pub_->get_topic_name()) > 0) {
      if (estimated_path_.poses.size() > cxt_.keep_poses_) {
        estimated_path_.poses.clear();
      }
      estimate_.add_to_path(estimated_path_);
      esimated_path_pub_->publish(estimated_path_);
    }

    // Advance plan and compute efforts
    Efforts efforts;
    if (mission_->advance(d, estimate_, efforts)) {
      // Publish control message
      publish_control(msg_time, efforts);

      if (mission_->status().planner == orca_msgs::msg::Control::PLAN_LOCAL &&
          count_subscribers(planned_pose_pub_->get_topic_name()) > 0) {
        // Publish planned pose for visualization
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = cxt_.map_frame_;
        pose_msg.pose = mission_->status().pose.fp.pose.pose.to_msg();
        planned_pose_pub_->publish(pose_msg);
      }
    } else {
      // Mission is over, clean up
      mission_ = nullptr;
      publish_control(msg_time, {});
    }
  }

  void AUVNode::publish_control(const rclcpp::Time &msg_time, const orca::Efforts &efforts)
  {
    // Combine joystick efforts to get thruster efforts.
    std::vector<double> thruster_efforts;
    for (const auto &i : THRUSTERS) {
      // Clamp forward + strafe to xy_gain_
      double xy_effort = clamp(
        efforts.forward() * i.forward_factor + efforts.strafe() * i.strafe_factor,
        -cxt_.xy_gain_, cxt_.xy_gain_);

      // Clamp total thrust
      thruster_efforts.push_back(
        clamp(xy_effort + efforts.yaw() * i.yaw_factor + efforts.vertical() * i.vertical_factor,
              THRUST_FULL_REV, THRUST_FULL_FWD));
    }

    orca_msgs::msg::Control control_msg;
    control_msg.header.stamp = msg_time;
    control_msg.header.frame_id = cxt_.base_frame_; // Control is expressed in the base frame

    // Diagnostics
    control_msg.mode = orca_msgs::msg::Control::AUV;
    control_msg.global_plan_idx = global_plan_idx_;
    if (mission_) {
      auto status = mission_->status();
      control_msg.targets_total = status.targets_total;
      control_msg.target_idx = status.target_idx;
      control_msg.target_marker_id = status.target_marker_id;
      control_msg.planner = status.planner;
      control_msg.local_plan_idx = status.local_plan_idx;
      control_msg.segments_total = status.segments_total;
      control_msg.segment_idx = status.segment_idx;
      control_msg.segment_info = status.segment_info;
      control_msg.segment_type = status.segment_type;
      control_msg.plan_pose = status.pose.fp.pose.pose.to_msg();
      control_msg.plan_twist = status.twist.to_msg();
    }
    control_msg.estimate_pose = estimate_.fp.pose.pose.to_msg();
    control_msg.good_pose = estimate_.fp.good_pose(cxt_.good_pose_dist_);
    control_msg.good_z = estimate_.fp.good_z();
    control_msg.has_good_observation = estimate_.fp.has_good_observation(cxt_.good_obs_dist_);
    control_msg.efforts = efforts.to_msg();
    control_msg.stability = 1.0;
    control_msg.odom_lag = (now() - msg_time).seconds();

    // Control
    control_msg.camera_tilt_pwm = tilt_to_pwm(0);
    control_msg.brightness_pwm = brightness_to_pwm(0);
    for (double thruster_effort : thruster_efforts) {
      control_msg.thruster_pwm.push_back(effort_to_pwm(thruster_effort));
    }
    control_pub_->publish(control_msg);
  }

  void AUVNode::write_status(cv::Mat &image)
  {
    cv::Point2d p{10, 20};
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);

    // Line 1: mission status
    if (mission_) {
      ss << "MISSION target m" << mission_->status().target_marker_id;
      if (mission_->status().planner == orca_msgs::msg::Control::PLAN_RECOVERY_MTM) {
        ss << " RECOVERY move to m" << mission_->status().target_marker_id;
      } else {
        ss << " plan z=" << mission_->status().pose.fp.pose.pose.z;
      }
    } else {
      ss << "WAITING z=" << base_link_z_;
    }
    draw_text(image, ss.str(), p, CV_RGB(255, 0, 0));

    // Line 2: pose estimate
    p.y += 20;
    ss.str("");
    if (estimate_.fp.good_pose(cxt_.good_pose_dist_)) {
      ss << "GOOD POSE";
    } else if (estimate_.fp.has_good_observation(cxt_.good_obs_dist_)) {
      ss << "GOOD OBSERVATION";
    } else if (!estimate_.fp.observations.empty()) {
      ss << "TOO FAR AWAY";
    } else {
      ss << "NO OBSERVATIONS";
    }
    ss << " z=" << base_link_z_;

    draw_text(image, ss.str(), p, CV_RGB(0, 255, 0));
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
  auto node = std::make_shared<orca_base::AUVNode>();

  // Set logger level
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  (void) result;

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
