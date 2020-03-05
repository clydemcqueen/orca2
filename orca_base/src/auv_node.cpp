#include "orca_base/auv_node.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "orca_shared/pwm.hpp"
#include "orca_base/thrusters.hpp"

/*******************************************************************************************************
 * There are several cxt_ options in auv_node and filter_node that interact. 2 examples:
 *
 * Without a filter:
 *      Launch auv_node
 *      AUVContext:
 *          loop_driver = 0 (timer driven)
 *          filtered_odom = false
 *          fuse_depth = true
 *          publish_tf = true
 *
 * With a filter:
 *      Launch auv_node and filter_node
 *      AUVContext:
 *          loop_driver = 2 (fiducial driven)
 *          filtered_odom = true
 *          fuse_depth = false
 *          publish_tf = false
 *      FilterContext:
 *          filter_baro = true
 *          filter_fcam = true
 *          publish_tf = true
 *
 * Other options are possible, e.g., filter_baro = false and fuse_depth = true
 */

using namespace orca;

namespace orca_base
{

  //=============================================================================
  // AUVNode
  //=============================================================================

  constexpr int TIMER_DRIVEN = 0;
  constexpr int DEPTH_DRIVEN = 1;
  constexpr int FIDUCIAL_DRIVEN = 2;

  constexpr int QUEUE_SIZE = 10;

  AUVNode::AUVNode() : Node{"auv_node"}, map_{get_logger(), cxt_}
  {
    // Suppress IDE warnings
    (void) battery_sub_;
    (void) depth_sub_;
    (void) fcam_info_sub_;
    (void) goal_sub_;
    (void) leak_sub_;
    (void) map_sub_;
    (void) obs_sub_;
    (void) spin_timer_;
    (void) mission_server_;

    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(AUV_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), AUV_NODE_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    AUV_NODE_ALL_PARAMS

    // Publications
    control_pub_ = create_publisher<orca_msgs::msg::Control>("control", QUEUE_SIZE);
    esimated_path_pub_ = create_publisher<nav_msgs::msg::Path>("filtered_path", QUEUE_SIZE);
    planned_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("planned_pose", QUEUE_SIZE);
    target_path_pub_ = create_publisher<nav_msgs::msg::Path>("target_path", QUEUE_SIZE);
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", QUEUE_SIZE);

    // Camera info may be published with a different QoS
    auto camera_info_qos = rclcpp::QoS{rclcpp::SensorDataQoS()};

    // Monotonic subscriptions
    depth_sub_ = create_subscription<orca_msgs::msg::Depth>(
      "depth", QUEUE_SIZE, [this](const orca_msgs::msg::Depth::SharedPtr msg) -> void
      { this->depth_cb_.call(msg); });
    fcam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "fcam_info", camera_info_qos, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) -> void
      { this->fcam_info_cb_.call(msg); });
    map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
      "fiducial_map", QUEUE_SIZE, [this](const fiducial_vlam_msgs::msg::Map::SharedPtr msg) -> void
      { this->map_cb_.call(msg); });

    using namespace std::placeholders;

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
    // Sync subscriptions
    if (cxt_.filtered_odom_) {
      // Message filter subscriptions
      obs_sub_.subscribe(this, "fiducial_observations"); // Default qos is reliable, queue=10
      fcam_pose_sub_.unsubscribe();
      odom_sub_.subscribe(this, "odom"); // Default qos is reliable, queue=10

      // Stop raw sync
      raw_sync_ = nullptr;

      // Start filtered sync
      using namespace std::placeholders;
      filtered_sync_.reset(new FilteredSync(FilteredPolicy(QUEUE_SIZE), obs_sub_, odom_sub_));
      filtered_sync_->registerCallback(std::bind(&AUVNode::filtered_fiducial_callback, this, _1, _2));
    } else {
      // Message filter subscriptions
      obs_sub_.subscribe(this, "fiducial_observations"); // Default qos is reliable, queue=10
      fcam_pose_sub_.subscribe(this, "fcam_f_map"); // Default qos is reliable, queue=10
      odom_sub_.unsubscribe();

      // Stop filtered sync
      filtered_sync_ = nullptr;

      // Start raw sync
      using namespace std::placeholders;
      raw_sync_.reset(new RawSync(RawPolicy(QUEUE_SIZE), obs_sub_, fcam_pose_sub_));
      raw_sync_->registerCallback(std::bind(&AUVNode::raw_fiducial_callback, this, _1, _2));
    }

    // Update model from new parameters
    cxt_.model_.fluid_density_ = cxt_.fluid_density_;
    cxt_.model_.bollard_force_xy_ = cxt_.bollard_force_xy_;
    cxt_.model_.bollard_force_z_ = cxt_.bollard_force_z_;
    cxt_.model_.t200_max_pos_force_ = cxt_.t200_max_pos_force_;
    cxt_.model_.t200_max_neg_force_ = cxt_.t200_max_neg_force_;
    cxt_.model_.drag_coef_f_ = cxt_.drag_coef_f_;
    cxt_.model_.drag_coef_s_ = cxt_.drag_coef_s_;
    cxt_.model_.drag_coef_z_ = cxt_.drag_coef_z_;
    cxt_.model_.drag_coef_tether_ = cxt_.drag_coef_tether_;
    cxt_.model_.drag_partial_const_yaw_ = cxt_.drag_partial_const_yaw_;

    // Update timer period and timeouts
    spin_period_ = std::chrono::milliseconds{cxt_.timer_period_ms_};
    depth_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_depth_ms_)};
    fp_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_fp_ms_)};

    // [Re-]start loop
    // Loop will run at ~constant wall speed, switch to ros_timer when it exists
    spin_timer_ = create_wall_timer(spin_period_, [this]() -> void
    { this->timer_cb_.call(); });
  }

  bool AUVNode::depth_ok(const rclcpp::Time &t)
  { return depth_cb_.receiving() && t - depth_cb_.prev() < depth_timeout_; }

  bool AUVNode::fp_ok(const rclcpp::Time &t)
  { return monotonic::valid(estimate_.t) && t - estimate_.t < fp_timeout_; }

  bool AUVNode::cam_info_ok()
  { return fcam_info_cb_.receiving(); }

  bool AUVNode::ready_to_start_mission()
  {
    // -- not currently in a mission
    // -- have a map
    // -- have camera info
    // -- getting depth and fiducial messages
    // -- have a good pose or a good observation
    return !mission_ && map_.ok() && cam_info_ok() && depth_ok(now()) && fp_ok(now()) &&
           (estimate_.fp.good_pose(cxt_.good_pose_dist_) || estimate_.fp.has_good_observation(cxt_.good_obs_dist_));
  }

  void AUVNode::timer_callback(bool first)
  {
    // Skip first time through the loop
    if (first) {
      return;
    }

    auto curr_time = timer_cb_.curr();
    auto prev_time = timer_cb_.prev();

    // If we're expecting steady depth messages, but they stop, then abort the mission
    if (mission_ && (cxt_.loop_driver_ == DEPTH_DRIVEN || cxt_.fuse_depth_) && !depth_ok(curr_time)) {
      RCLCPP_ERROR(get_logger(), "lost depth readings, abort mission");
      abort_mission(curr_time);
    }

    // If we're expecting steady fiducial messages, but they stop, then abort the mission
    if (mission_ && cxt_.loop_driver_ == FIDUCIAL_DRIVEN && !fp_ok(curr_time)) {
      RCLCPP_ERROR(get_logger(), "lost fiducial readings, abort mission");
      abort_mission(curr_time);
    }

    // Drive the AUV loop
    if (mission_ && cxt_.loop_driver_ == TIMER_DRIVEN) {
      auv_advance(curr_time, curr_time - prev_time);
    }
  }

  void AUVNode::battery_callback(const orca_msgs::msg::Battery::SharedPtr msg)
  {
    if (mission_ && msg->low_battery) {
      RCLCPP_ERROR(get_logger(), "low battery (%g volts), abort mission", msg->voltage);
      abort_mission(msg->header.stamp);
    }
  }

  void AUVNode::depth_callback(orca_msgs::msg::Depth::SharedPtr msg, bool first)
  {
    // Skip the first message
    if (first) {
      return;
    }

    // Save latest depth reading
    base_link_z_ = msg->z;

    // Drive the AUV loop
    if (mission_ && cxt_.loop_driver_ == DEPTH_DRIVEN) {
      auv_advance(depth_cb_.curr(), depth_cb_.d());
    }
  }

  void AUVNode::fcam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fcam_model_.fromCameraInfo(msg);
  }

  void AUVNode::leak_callback(const orca_msgs::msg::Leak::SharedPtr msg)
  {
    if (mission_ && msg->leak_detected) {
      RCLCPP_ERROR(get_logger(), "leak detected, abort mission");
      abort_mission(msg->header.stamp);
    }
  }

  void AUVNode::map_callback(const fiducial_vlam_msgs::msg::Map::SharedPtr msg)
  {
    map_.set_vlam_map(msg);
  }

  // Get a synchronized set of messages from vloc: pose from SolvePnP and raw marker observations
  // Guard against invalid or duplicate timestamps
  // vloc poses are sensor_f_map -- transform to base_f_map
  void AUVNode::raw_fiducial_callback(
    const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &obs_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &fcam_msg)
  {
    // Keep track of the previous time
    static rclcpp::Time prev_time{0, 0, RCL_ROS_TIME};

    // Wait for at least 1 map and 1 depth message
    if (!map_.ok() || !depth_cb_.receiving()) {
      return;
    }

    // The current time
    rclcpp::Time curr_time = obs_msg->header.stamp;

    // Skip first message
    if (!monotonic::valid(prev_time)) {
      prev_time = curr_time;
      return;
    }

    // Ignore invalid timestamps
    if (!monotonic::valid(curr_time)) {
      return;
    }

    // Ignore duplicates, or time going backwards
    // (Possible in simulation if we're running ROS time, see notes in orca_gazebo README)
    if (curr_time <= prev_time) {
      return;
    }

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
      transform.header.stamp = curr_time;
      transform.header.frame_id = cxt_.map_frame_;
      transform.child_frame_id = cxt_.base_frame_;
      transform.transform = toMsg(t_map_base);

      // TF messages can have multiple transforms, we have just 1
      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.emplace_back(transform);

      tf_pub_->publish(tf_message);
    }

    // Save the observations and pose
    estimate_.from_msgs(*obs_msg, base_f_map, map_.marker_length(), cxt_.fcam_hfov_, cxt_.fcam_hres_);

    // Drive the AUV loop
    if (mission_ && cxt_.loop_driver_ == FIDUCIAL_DRIVEN) {
      auv_advance(estimate_.t, curr_time - prev_time);
    }

    // Update prev_time
    prev_time = curr_time;
  }

  // Get a synchronized set of messages: raw marker observations from vloc and filtered odom from filter_node
  // filter_node guarantees that all messages are monotonic, no need to guard against invalid or duplicate timestamps
  // filter_node odom messages are base_f_map
  void AUVNode::filtered_fiducial_callback(
    const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &obs_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
  {
    // Keep track of the previous time
    static rclcpp::Time prev_time{0, 0, RCL_ROS_TIME};

    // Wait for at least 1 map and 1 depth message
    if (!map_.ok() || !depth_cb_.receiving()) {
      return;
    }

    // The current time
    rclcpp::Time curr_time = obs_msg->header.stamp;

    // Skip first message
    if (!monotonic::valid(prev_time)) {
      prev_time = curr_time;
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped base_f_map;
    base_f_map.header = odom_msg->header;
    base_f_map.pose = odom_msg->pose;

    // Publish tf
    if (cxt_.publish_tf_ && tf_pub_->get_subscription_count() > 0) {
      // Can't convert geometry_msgs::msg::Pose to geometry_msgs::msg::Transform, so go by way of tf2::Transform
      tf2::Transform t_map_base;
      tf2::fromMsg(odom_msg->pose.pose, t_map_base);

      // Build transform message
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = curr_time;
      transform.header.frame_id = cxt_.map_frame_;
      transform.child_frame_id = cxt_.base_frame_;
      transform.transform = toMsg(t_map_base);

      // TF messages can have multiple transforms, we have just 1
      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.emplace_back(transform);

      tf_pub_->publish(tf_message);
    }

    // Save the observations and pose
    estimate_.from_msgs(*obs_msg, base_f_map, map_.marker_length(), cxt_.fcam_hfov_, cxt_.fcam_hres_);

    // Drive the AUV loop
    if (mission_ && cxt_.loop_driver_ == FIDUCIAL_DRIVEN) {
      auv_advance(estimate_.t, curr_time - prev_time);
    }

    // Update prev_time
    prev_time = curr_time;
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

  /***********************************************************************************
   * Start a mission. There are lots of options! Here are some default cases:
   *
   * If !pose_targets, then look to markers for a list of markers
   * If markers.empty(), then use all markers in the map
   *
   * If pose_targets, then look to poses for a list of poses
   * If poses.empty(), then try to provide the current pose
   * This only works if we have a good pose. It's only useful if keep_station is true
   *
   * @param goal_handle Action goal
   */
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

  /**************************************************************************************
   * Primary entry point for AUV operation. This calls Mission::advance(), etc.
   *
   * cxt_.loop_driver specifies one of 3 loop drivers:
   *
   * TIMER_DRIVEN         Called by timer_callback
   *                      Pros:
   *                      -- provides a consistent 20Hz rate (depends on cxt_.timer_period_ms_)
   *
   * DEPTH_DRIVEN         Called by depth_callback
   *                      Pros:
   *                      -- no lag between new depth data & controller response
   *                      -- should provide a consistent 10Hz rate [but messages may be lost or delayed]
   *
   * FIDUCIAL_DRIVEN      Called by fiducial_callback
   *                      Pros:
   *                      -- no lag between new fiducial data & controller response
   *                      -- if orca_filter is running upstream and fusing depth and fiducial data, should provide
   *                         a consistent stream of messages at 10-40Hz [but messages may be lost or delayed]
   *
   * The gating factor for depth messages is the barometer driver in orca_driver: it takes at least 40ms
   * to get a good reading. 10Hz seems like a reasonable rate.
   *
   * The image message rates depend on many factors, but 15-30Hz is typical.
   *
   * orca_filter runs at the sum of all of the sensor rates, so 1 camera at 15fps + 1 barometer at 10Hz
   * should provide 25Hz when there's a marker in view, and 10Hz when there isn't.
   *
   * The planners require that curr_time = prev_time + d, so we can't limit d.
   *
   * @param msg_time Time of call, either sensor_msg.header.stamp or now()
   * @param d dt since last call
   */
  void AUVNode::auv_advance(const rclcpp::Time &msg_time, const rclcpp::Duration &d)
  {
    if (d.seconds() > 0.2) {
      RCLCPP_WARN(get_logger(), "auv loop d=%g seconds higher than expected", d.seconds());
    }

    // Fuse depth and fiducial messages
    // Be sure to turn this off if there is upstream filter fusing this data
    if (cxt_.fuse_depth_) {
      estimate_.fp.set_good_z(base_link_z_);
    }

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
      for (auto &obs : status.pose.fp.observations) {
        control_msg.plan_observations.push_back(obs.to_msg());
      }
    }
    control_msg.estimate_pose = estimate_.fp.pose.pose.to_msg();
    for (auto &obs : estimate_.fp.observations) {
      control_msg.estimate_observations.push_back(obs.to_msg());
    }
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
