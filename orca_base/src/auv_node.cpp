// Copyright (c) 2020, Clyde McQueen.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "orca_base/auv_node.hpp"

#include <memory>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "orca_shared/pwm.hpp"

/***************************************************************************************************
 * AUVNode can run "dead reckoning" when there are no markers in sight. Caveats:
 * -- depth information is required, either by overriding the z position with depth, or by fusing
 *    depth information in the filter.
 * -- the BlueROV2 sub won't hold a constant heading, so this only works for a few meters.
 *
 * These cases are supported:
 * -- case 1: filter_node is running and fusing depth
 * -- case 2: filter_node is running and not fusing depth; auv_node overrides depth
 * -- case 3: filter_node is not running, auv_node overrides depth
 *
 * Note that exactly one node should publish map=>base tf.
 *
 * Case 1 example:
 *      fp_node:
 *          publish_tf = false
 *      filter_node:
 *          filter_baro = true
 *          filter_fcam = true
 *          publish_tf = true
 *      auv_node:
 *          loop_driver = 2 (fiducial driven)
 *          depth_override = false
 *
 * Case 3 example:
 *      fp_node:
 *          publish_tf = true
 *      auv_node:
 *          loop_driver = 0 (timer driven)
 *          depth_override = true
 */

namespace orca_base
{

//=============================================================================
// AUVNode
//=============================================================================

constexpr int TIMER_DRIVEN = 0;
constexpr int DEPTH_DRIVEN = 1;
constexpr int FIDUCIAL_DRIVEN = 2;

constexpr int QUEUE_SIZE = 10;

AUVNode::AUVNode()
: Node{"auv_node"}
{
  // Suppress IDE warnings
  (void) control_sub_;
  (void) depth_sub_;
  (void) driver_sub_;
  (void) fp_sub_;
  (void) goal_sub_;
  (void) map_sub_;
  (void) spin_timer_;
  (void) mission_server_;

  // Get parameters, this will immediately call validate_parameters()
  // TODO(clyde): catch rclcpp::ParameterTypeException
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

  // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
  CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), AUV_NODE_ALL_PARAMS)

  // Publications
  control_pub_ = create_publisher<orca_msgs::msg::Control>("control", QUEUE_SIZE);
  esimated_path_pub_ = create_publisher<nav_msgs::msg::Path>("filtered_path", QUEUE_SIZE);
  planned_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("planned_pose", QUEUE_SIZE);
  target_path_pub_ = create_publisher<nav_msgs::msg::Path>("target_path", QUEUE_SIZE);

  // Camera info may be published with a different QoS
  auto camera_info_qos = rclcpp::QoS{rclcpp::SensorDataQoS()};

  // Monotonic subscriptions
  driver_sub_ = create_subscription<orca_msgs::msg::Driver>(
    "driver_status", QUEUE_SIZE, [this](const orca_msgs::msg::Driver::SharedPtr msg) -> void
    {this->driver_cb_.call(msg);});
  fp_sub_ = create_subscription<orca_msgs::msg::FiducialPoseStamped>(
    "filtered_fp", QUEUE_SIZE,
    [this](const orca_msgs::msg::FiducialPoseStamped::SharedPtr msg) -> void
    {this->fp_cb_.call(msg);});
  map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
    "fiducial_map", QUEUE_SIZE, [this](const fiducial_vlam_msgs::msg::Map::SharedPtr msg) -> void
    {this->map_cb_.call(msg);});

  using namespace std::placeholders;

  // Other subscriptions
  auto control_cb = std::bind(&AUVNode::control_callback, this, _1);
  control_sub_ =
    create_subscription<orca_msgs::msg::Control>("rov_control", QUEUE_SIZE, control_cb);

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

  RCLCPP_INFO(get_logger(), "auv_node ready");
}

AUVNode::~AUVNode()
{
  // Abort mission!
  abort_mission(now());
}

void AUVNode::validate_parameters()
{
  // Subscribe to depth messages
  if (cxt_.loop_driver_ == DEPTH_DRIVEN || cxt_.depth_override_) {
    depth_sub_ = create_subscription<orca_msgs::msg::Depth>(
      "depth", QUEUE_SIZE, [this](const orca_msgs::msg::Depth::SharedPtr msg) -> void
      {this->depth_cb_.call(msg);});
  } else {
    depth_sub_.reset();
  }

  // Update model from new parameters
  cxt_.model_.mass_ = cxt_.mass_;
  cxt_.model_.volume_ = cxt_.volume_;
  cxt_.model_.fluid_density_ = cxt_.fluid_density_;
  cxt_.model_.bollard_force_xy_ = cxt_.bollard_force_xy_;
  cxt_.model_.bollard_force_z_up_ = cxt_.bollard_force_z_up_;
  cxt_.model_.bollard_force_z_down_ = cxt_.bollard_force_z_down_;
  cxt_.model_.max_torque_yaw_ = cxt_.max_torque_yaw_;
  cxt_.model_.drag_coef_f_ = cxt_.drag_coef_f_;
  cxt_.model_.drag_coef_s_ = cxt_.drag_coef_s_;
  cxt_.model_.drag_coef_z_ = cxt_.drag_coef_z_;
  cxt_.model_.drag_coef_tether_ = cxt_.drag_coef_tether_;
  cxt_.model_.drag_partial_const_yaw_ = cxt_.drag_partial_const_yaw_;

  // Update timeouts
  depth_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_depth_ms_)};
  driver_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_driver_ms_)};
  fp_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_fp_ms_)};

  // [Re-]start loop
  // Loop will run at ~constant wall speed, switch to ros_timer when it exists
  spin_timer_ = create_wall_timer(std::chrono::milliseconds{cxt_.timer_period_ms_}, [this]() -> void
      {this->timer_cb_.call();});

  double hover_accel = cxt_.model_.hover_accel_z();
  double hover_effort = cxt_.model_.accel_to_effort_z(hover_accel);
  RCLCPP_INFO(get_logger(), "hover accel: %g, effort: %g, pwm: %d",
    hover_accel, hover_effort, orca::effort_to_pwm(hover_effort));
}

bool AUVNode::depth_ok(const rclcpp::Time & t)
{
  return depth_cb_.receiving() && t - depth_cb_.prev() < depth_timeout_;
}

bool AUVNode::driver_ok(const rclcpp::Time & t)
{
  return driver_cb_.receiving() && t - driver_cb_.prev() < driver_timeout_;
}

bool AUVNode::fp_ok(const rclcpp::Time & t)
{
  return estimate_.header().valid() && t - estimate_.header().t() < fp_timeout_;
}

bool AUVNode::accept_goal(const rclcpp::Time & t)
{
  if (mission_) {
    RCLCPP_ERROR(get_logger(), "goal rejected: mission already running");
    return false;
  }

  if (!map_.valid()) {
    RCLCPP_ERROR(get_logger(), "goal rejected: no map");
    return false;
  }

  if (!driver_ok(t)) {
    RCLCPP_ERROR(get_logger(), "goal rejected: no driver status");
    return false;
  }

  if ((cxt_.loop_driver_ == DEPTH_DRIVEN || cxt_.depth_override_) && !depth_ok(t)) {
    RCLCPP_ERROR(get_logger(), "goal rejected: no depth messages");
    return false;
  }

  if (!fp_ok(t)) {
    RCLCPP_ERROR(get_logger(), "goal rejected: no fiducial pose messages");
    return false;
  }

  if (!(estimate_.fp().good(cxt_.good_pose_dist_) ||
    estimate_.fp().observations().good(cxt_.good_obs_dist_)))
  {
    RCLCPP_ERROR(get_logger(), "goal rejected: no good pose, no good observation");
    return false;
  }

  RCLCPP_INFO(get_logger(), "goal accepted");
  return true;
}

void AUVNode::timer_callback(bool first)
{
  // Skip first time through the loop
  if (first) {
    return;
  }

  auto curr_time = timer_cb_.curr();

  // If the driver stopped sending status messages, then abort the mission
  if (mission_ && !driver_ok(curr_time)) {
    RCLCPP_ERROR(get_logger(), "lost driver messages, abort mission");
    abort_mission(curr_time);
  }

  // If we're expecting steady depth messages, but they stop, then abort the mission
  if (mission_ && (cxt_.loop_driver_ == DEPTH_DRIVEN || cxt_.depth_override_) &&
    !depth_ok(curr_time))
  {
    RCLCPP_ERROR(get_logger(), "lost depth messages, abort mission");
    abort_mission(curr_time);
  }

  // If we're expecting steady fiducial messages, but they stop, then abort the mission
  if (mission_ && !fp_ok(curr_time) &&
    (cxt_.loop_driver_ == FIDUCIAL_DRIVEN || !cxt_.depth_override_))
  {
    RCLCPP_ERROR(get_logger(), "lost fiducial pose messages, abort mission");
    abort_mission(curr_time);
  }

  // If we stopped getting fiducial messages, zero out estimate_
  if (!fp_ok(curr_time)) {
    estimate_ = {};
  }

  // Drive the AUV loop
  if (mission_ && cxt_.loop_driver_ == TIMER_DRIVEN) {
    auv_advance(curr_time, timer_cb_.d());
  }
}

void AUVNode::control_callback(const orca_msgs::msg::Control::SharedPtr msg)
{
  if (mission_) {
    RCLCPP_INFO(get_logger(), "received ROV message, aborting mission");
    abort_mission(msg->header.stamp);
  }

  control_pub_->publish(*msg);
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

void AUVNode::driver_callback(const orca_msgs::msg::Driver::SharedPtr msg)
{
  if (mission_ && !(msg->status == orca_msgs::msg::Driver::STATUS_OK ||
    msg->status == orca_msgs::msg::Driver::STATUS_OK_MISSION))
  {
    RCLCPP_ERROR(get_logger(), "driver problem, abort mission");
    abort_mission(msg->header.stamp);
  }
}

void AUVNode::map_callback(const fiducial_vlam_msgs::msg::Map::SharedPtr msg)
{
  map_ = mw::Map{*msg};
}

void AUVNode::fp_callback(orca_msgs::msg::FiducialPoseStamped::SharedPtr msg, bool first)
{
  // Skip first message
  if (first) {
    return;
  }

  estimate_ = mw::FiducialPoseStamped{*msg};

  // Drive the AUV loop
  if (mission_ && cxt_.loop_driver_ == FIDUCIAL_DRIVEN) {
    auv_advance(estimate_.header().t(), fp_cb_.d());
  }
}

rclcpp_action::GoalResponse AUVNode::mission_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MissionAction::Goal> goal)
{
  (void) goal;

  using orca_msgs::msg::Control;

  if (accept_goal(now())) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  } else {
    return rclcpp_action::GoalResponse::REJECT;
  }
}

rclcpp_action::CancelResponse
AUVNode::mission_cancel(const std::shared_ptr<MissionHandle> goal_handle)
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

  RCLCPP_INFO(get_logger(),
    "start mission, pose_targets=%d, %d marker(s), %d pose(s), random=%d, keep=%d",
    action->pose_targets, action->marker_ids.size(), action->poses.size(), action->random,
    action->keep_station);

  // Create a global planner
  ++global_plan_idx_;
  std::shared_ptr<GlobalPlanner> planner;
  if (action->pose_targets) {
    // If poses.empty() then go to the current pose
    if (action->poses.empty() && action->keep_station &&
      estimate_.fp().good(cxt_.good_pose_dist_))
    {
      RCLCPP_INFO(get_logger(), "keeping station at current pose");
      std::vector<geometry_msgs::msg::Pose> poses;
      poses.push_back(estimate_.fp().pose().pose().msg());
      planner = GlobalPlanner::plan_poses(get_logger(), cxt_, map_,
          estimate_.fp().observations().observer(), action->mission_info, poses,
          action->random, action->keep_station);
    } else {
      planner = GlobalPlanner::plan_poses(get_logger(), cxt_, map_,
          estimate_.fp().observations().observer(), action->mission_info,
          action->poses, action->random, action->keep_station);
    }
  } else {
    planner = GlobalPlanner::plan_markers(get_logger(), cxt_, map_,
        estimate_.fp().observations().observer(), action->mission_info,
        action->marker_ids, action->random, action->keep_station);
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

void AUVNode::abort_mission(const rclcpp::Time & msg_time)
{
  if (mission_) {
    mission_->abort();
    mission_.reset();

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
 *                      -- should provide a consistent ~20Hz rate [but messages may be lost or delayed]
 *
 * FIDUCIAL_DRIVEN      Called by fp_callback
 *                      Pros:
 *                      -- no lag between new fiducial data & controller response
 *                      -- if filter_node is running upstream and fusing depth and fiducial data, should provide
 *                         a consistent stream of messages at 10-40Hz [but messages may be lost or delayed]
 *
 * The gating factor for depth messages is the barometer driver in orca_driver: it takes at least 40ms
 * to get a good reading.
 *
 * The image message rates depend on many factors, but 15-30Hz is typical.
 *
 * filter_node can run at the sum of all of the sensor rates, so 1 camera at 30fps + 1 barometer at 20Hz
 * might provide 50Hz when there's a marker in view, and 20Hz when there isn't.
 *
 * The planners require that curr_time = prev_time + d, so we can't limit d.
 *
 * @param t Time of call, either sensor_msg.header.stamp or now()
 * @param d dt since last call
 */
void AUVNode::auv_advance(const rclcpp::Time & t, const rclcpp::Duration & d)
{
  if (d.seconds() > 0.2) {
    RCLCPP_WARN(get_logger(), "high dt: %g > 0.2", d.seconds());
  }

  // Depth overrides pose.position.z
  if (cxt_.depth_override_) {
    auto msg = estimate_.msg();
    msg.fp.pose.pose.position.z = base_link_z_;
    estimate_ = mw::FiducialPoseStamped{msg};
  }

  // Publish estimated path
  if (estimate_.fp().good(cxt_.good_pose_dist_) &&
    count_subscribers(esimated_path_pub_->get_topic_name()) > 0)
  {
    if (estimated_path_.poses.size() > cxt_.keep_poses_) {
      estimated_path_.poses.clear();
    }
    estimate_.add_to_path(estimated_path_);
    esimated_path_pub_->publish(estimated_path_);
  }

  // Advance plan and compute efforts
  mw::Efforts efforts;
  if (mission_->advance(d, estimate_, efforts)) {
    // Publish control message
    publish_control(t, efforts);

    if (mission_->status().planner() == orca_msgs::msg::MissionState::PLAN_LOCAL &&
      count_subscribers(planned_pose_pub_->get_topic_name()) > 0)
    {
      // Publish planned pose for visualization
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.frame_id = cxt_.map_frame_;
      pose_msg.pose = mission_->status().plan().fp().pose().pose().msg();
      planned_pose_pub_->publish(pose_msg);
    }
  } else {
    // Mission is over, clean up
    mission_.reset();
    publish_control(t, {});
  }
}

void AUVNode::publish_control(const rclcpp::Time & msg_time, const mw::Efforts & efforts)
{
  orca_msgs::msg::Control control_msg;
  control_msg.header.stamp = msg_time;
  control_msg.header.frame_id = cxt_.base_frame_;  // Control is expressed in the base frame

  // Diagnostics
  control_msg.mode = orca_msgs::msg::Control::AUV;
  control_msg.global_plan_idx = global_plan_idx_;
  if (mission_) {
    control_msg.mission = mission_->status().msg();
  }
  control_msg.estimate = estimate_.fp().msg();
  control_msg.good_pose = estimate_.fp().good(cxt_.good_pose_dist_);
  control_msg.covariance_dof = estimate_.fp().pose().dof();
  control_msg.has_good_observation = estimate_.fp().observations().good(cxt_.good_obs_dist_);
  control_msg.efforts = efforts.to_msg();
  control_msg.odom_lag = (now() - estimate_.header().t()).seconds();

  // Control
  control_msg.camera_tilt_pwm = orca::tilt_to_pwm(0);
  control_msg.brightness_pwm = orca::brightness_to_pwm(0);
  thrusters_.efforts_to_control(efforts, cxt_.xy_limit_, cxt_.thruster_accel_limit_, control_msg);

  control_pub_->publish(control_msg);
}

}  // namespace orca_base

//=============================================================================
// Main
//=============================================================================

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_base::AUVNode>();

  // Set logger level
  auto result =
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  (void) result;

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
