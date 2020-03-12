#include "orca_base/rov_node.hpp"

#include <iomanip>

#include "orca_shared/pwm.hpp"

#include "orca_base/thrusters.hpp"

using namespace orca;

namespace orca_base
{

  //=============================================================================
  // Utilities
  //=============================================================================

  // Sense a button down event
  bool button_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int button)
  {
    return curr->buttons[button] && !prev.buttons[button];
  }

  // Sense a trim down event
  bool trim_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int axis)
  {
    return curr->axes[axis] && !prev.axes[axis];
  }

  constexpr bool is_hold_pressure_mode(uint8_t mode)
  {
    using orca_msgs::msg::Control;
    return mode == Control::ROV_HOLD_PRESSURE;
  }

  constexpr bool is_rov_mode(uint8_t mode)
  {
    using orca_msgs::msg::Control;
    return mode == Control::ROV || mode == Control::ROV_HOLD_PRESSURE;
  }

  constexpr bool is_auv_mode(uint8_t mode)
  {
    using orca_msgs::msg::Control;
    return mode == Control::AUV;
  }

  //=============================================================================
  // ROVNode
  //=============================================================================

  ROVNode::ROVNode() : Node{"rov_node"}
  {
    // Suppress IDE warnings
    (void) baro_sub_;
    (void) battery_sub_;
    (void) goal_sub_;
    (void) joy_sub_;
    (void) leak_sub_;
    (void) spin_timer_;

    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(ROV_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), ROV_NODE_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    ROV_NODE_ALL_PARAMS

    // ROV PID controller
    pressure_hold_pid_ = std::make_shared<pid::Controller>(false, cxt_.rov_pressure_pid_kp_, cxt_.rov_pressure_pid_ki_,
                                                           cxt_.rov_pressure_pid_kd_);

    // Publications
    control_pub_ = create_publisher<orca_msgs::msg::Control>("control", 1);

    // Camera info may be published with a different QoS
    auto camera_info_qos = rclcpp::QoS{rclcpp::SensorDataQoS()};

    // Monotonic subscriptions
    baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
      "barometer", 1, [this](const orca_msgs::msg::Barometer::SharedPtr msg) -> void
      { this->baro_cb_.call(msg); });
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1, [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void
      { this->joy_cb_.call(msg); });

    using namespace std::placeholders;

    // Other subscriptions
    auto battery_cb = std::bind(&ROVNode::battery_callback, this, _1);
    battery_sub_ = create_subscription<orca_msgs::msg::Battery>("battery", 1, battery_cb);
    auto goal_cb = std::bind(&ROVNode::goal_callback, this, _1);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1, goal_cb);
    auto leak_cb = std::bind(&ROVNode::leak_callback, this, _1);
    leak_sub_ = create_subscription<orca_msgs::msg::Leak>("leak", 1, leak_cb);

    // Mission action client
    mission_client_ = rclcpp_action::create_client<MissionAction>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "mission");

    RCLCPP_INFO(get_logger(), "rov_node ready");
  }

  void ROVNode::validate_parameters()
  {
    // Update model from new parameters
    cxt_.model_.fluid_density_ = cxt_.fluid_density_;
    cxt_.model_.bollard_force_z_up_ = cxt_.bollard_force_z_up_;
    cxt_.model_.bollard_force_z_down_ = cxt_.bollard_force_z_down_;

    // Update timeouts
    baro_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_baro_ms_)};
    joy_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_joy_ms_)};
    spin_period_ = std::chrono::milliseconds{cxt_.timer_period_ms_};

    // [Re-]start loop
    // Loop will run at ~constant wall speed, switch to ros_timer when it exists
    spin_timer_ = create_wall_timer(spin_period_, std::bind(&ROVNode::spin_once, this));
  }

  bool ROVNode::holding_pressure()
  { return is_hold_pressure_mode(mode_); }

  bool ROVNode::rov_mode()
  { return is_rov_mode(mode_); }

  bool ROVNode::auv_mode()
  { return is_auv_mode(mode_); }

  bool ROVNode::baro_ok(const rclcpp::Time &t)
  { return baro_cb_.receiving() && t - baro_cb_.prev() < baro_timeout_; }

  bool ROVNode::joy_ok(const rclcpp::Time &t)
  { return joy_cb_.receiving() && t - joy_cb_.prev() < joy_timeout_; }

  // New barometer reading
  void ROVNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg)
  {
    pressure_ = msg->pressure;
  }

  // New battery reading
  void ROVNode::battery_callback(const orca_msgs::msg::Battery::SharedPtr msg)
  {
    if (msg->low_battery) {
      RCLCPP_ERROR(get_logger(), "low battery (%g volts), disarming", msg->voltage);
      disarm(msg->header.stamp);
    }
  }

  // Timer callback
  void ROVNode::spin_once()
  {
    // Ignore 0
    auto spin_time = now();
    if (spin_time.nanoseconds() <= 0) {
      return;
    }

    // Various timeouts
    if (rov_mode() && !joy_ok(spin_time)) {
      RCLCPP_ERROR(get_logger(), "lost joystick during ROV operation, disarming");
      disarm(spin_time);
    }

    if (holding_pressure() && !baro_ok(spin_time)) {
      RCLCPP_ERROR(get_logger(), "lost barometer while holding pressure, disarming");
      disarm(spin_time);
    }
  }

  // Start a mission to move to a particular goal -- called by rviz2
  void ROVNode::goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (rov_mode()) {
      RCLCPP_INFO(get_logger(), "goal accepted");
      msg->pose.position.z = cxt_.planner_target_z_;
      start_mission(msg->header.stamp, Mission::GO_TO_POSE, msg->pose);
    } else {
      RCLCPP_ERROR(get_logger(), "cannot start mission");
    }
  }

  // New input from the gamepad
  void ROVNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg, bool first)
  {
    if (!first) {
      // Arm/disarm
      if (button_down(msg, joy_msg_, joy_button_disarm_)) {
        disarm(msg->header.stamp);
      } else if (button_down(msg, joy_msg_, joy_button_arm_)) {
        start_rov(msg->header.stamp);
      }

      // If we're disarmed, ignore everything else
      if (mode_ == orca_msgs::msg::Control::DISARMED) {
        joy_msg_ = *msg;
        return;
      }

      // Mode
      if (button_down(msg, joy_msg_, joy_button_rov_)) {
        start_rov(msg->header.stamp);
      } else if (button_down(msg, joy_msg_, joy_button_rov_hold_pressure_)) {
        if (baro_ok(msg->header.stamp)) {
          start_hold_pressure(msg->header.stamp);
        } else {
          RCLCPP_ERROR(get_logger(), "barometer not ready, cannot hold pressure");
        }
      } else if (button_down(msg, joy_msg_, joy_button_auv_keep_station_)) {
        start_mission(msg->header.stamp, Mission::KEEP_STATION);
      } else if (button_down(msg, joy_msg_, joy_button_auv_random_)) {
        start_mission(msg->header.stamp, Mission::RANDOM_MARKERS);
      }

      // Z trim
      if (holding_pressure() && trim_down(msg, joy_msg_, joy_axis_z_trim_)) {
        pressure_hold_pid_->set_target(msg->axes[joy_axis_z_trim_] > 0 ?
                                       pressure_hold_pid_->target() - cxt_.inc_pressure_ :
                                       pressure_hold_pid_->target() + cxt_.inc_pressure_);
        RCLCPP_INFO(get_logger(), "hold pressure at %g", pressure_hold_pid_->target());
      }

      // Camera tilt
      if (button_down(msg, joy_msg_, joy_button_tilt_up_)) {
        tilt_ = clamp(tilt_ + cxt_.inc_tilt_, TILT_MIN, TILT_MAX);
        RCLCPP_INFO(get_logger(), "tilt at %d", tilt_);
      } else if (button_down(msg, joy_msg_, joy_button_tilt_down_)) {
        tilt_ = clamp(tilt_ - cxt_.inc_tilt_, TILT_MIN, TILT_MAX);
        RCLCPP_INFO(get_logger(), "tilt at %d", tilt_);
      }

      // Lights
      if (button_down(msg, joy_msg_, joy_button_bright_)) {
        brightness_ = clamp(brightness_ + cxt_.inc_lights_, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
        RCLCPP_INFO(get_logger(), "lights at %d", brightness_);
      } else if (button_down(msg, joy_msg_, joy_button_dim_)) {
        brightness_ = clamp(brightness_ - cxt_.inc_lights_, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
        RCLCPP_INFO(get_logger(), "lights at %d", brightness_);
      }

      // Thrusters
      if (rov_mode()) {
        rov_advance(msg->header.stamp);
      }
    }

    joy_msg_ = *msg;
  }

  // Leak detector
  void ROVNode::leak_callback(const orca_msgs::msg::Leak::SharedPtr msg)
  {
    if (msg->leak_detected) {
      RCLCPP_ERROR(get_logger(), "leak detected, disarming");
      disarm(msg->header.stamp);
    }
  }

  void ROVNode::goal_response_callback(std::shared_future<MissionHandle::SharedPtr> future)
  {
    const auto &goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "mission rejected");
    } else {
      RCLCPP_INFO(get_logger(), "mission accepted");
      mode_ = orca_msgs::msg::Control::AUV;
    }
  }

  void ROVNode::feedback_callback(MissionHandle::SharedPtr,
                                  const std::shared_ptr<const MissionAction::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "feedback: %d out of %d", feedback->targets_completed, feedback->targets_total);
  }

  void ROVNode::result_callback(const MissionHandle::WrappedResult &result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "mission succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "mission aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "mission canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "unknown result code");
        return;
    }

    disarm(now());
  }

  void ROVNode::rov_advance(const rclcpp::Time &stamp)
  {
    double dt = joy_cb_.dt();

    Efforts efforts;
    efforts.set_forward(dead_band(joy_msg_.axes[joy_axis_forward_], cxt_.input_dead_band_) * cxt_.xy_gain_);
    efforts.set_strafe(dead_band(joy_msg_.axes[joy_axis_strafe_], cxt_.input_dead_band_) * cxt_.xy_gain_);
    efforts.set_yaw(dead_band(joy_msg_.axes[joy_axis_yaw_], cxt_.input_dead_band_) * cxt_.yaw_gain_);

    if (holding_pressure()) {
      efforts.set_vertical(
        cxt_.model_.accel_to_effort_z(-pressure_hold_pid_->calc(pressure_, dt) + cxt_.model_.hover_accel_z()));
    } else {
      efforts.set_vertical(dead_band(joy_msg_.axes[joy_axis_vertical_], cxt_.input_dead_band_) * cxt_.vertical_gain_);
    }

    publish_control(stamp, efforts);
  }

  void ROVNode::publish_control(const rclcpp::Time &msg_time, const orca::Efforts &efforts)
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

    // Publish control message
    orca_msgs::msg::Control control_msg;
    control_msg.header.stamp = msg_time;
    control_msg.header.frame_id = cxt_.base_frame_; // Control is expressed in the base frame
    control_msg.estimate_pose = geometry_msgs::msg::Pose{};
    control_msg.efforts = efforts.to_msg();
    control_msg.stability = 1.0;
    control_msg.odom_lag = (now() - msg_time).seconds();
    control_msg.mode = mode_;
    control_msg.plan_pose = geometry_msgs::msg::Pose{};
    control_msg.plan_twist = geometry_msgs::msg::Twist{};
    control_msg.targets_total = 0;
    control_msg.target_idx = 0;
    control_msg.target_marker_id = 0;
    control_msg.planner = orca_msgs::msg::Control::PLAN_NONE;
    control_msg.segments_total = 0;
    control_msg.segment_idx = 0;
    control_msg.segment_info = "";
    control_msg.camera_tilt_pwm = tilt_to_pwm(tilt_);
    control_msg.brightness_pwm = brightness_to_pwm(brightness_);
    for (double thruster_effort : thruster_efforts) {
      control_msg.thruster_pwm.push_back(effort_to_pwm(thruster_effort));
    }
    control_pub_->publish(control_msg);
  }

  /**************************************************************************************************
   *  Mode changes work like this:
   *  1. stop_mission sends a "cancel all goals" message to auv_node
   *  2. change mode
   *  3. publish_control sends a "stop thrusters, here's the new mode" message to orca_driver
   *  4. other stuff that must be done
   *
   *  Starting a mission is async, so change mode twice:
   *  1. disarm right away, this will stop sending /control messages to avoid conflict with auv_node
   *  2. set to AUV when the mission is successfully started
   **************************************************************************************************/

  void ROVNode::disarm(const rclcpp::Time &msg_time)
  {
    stop_mission();
    mode_ = orca_msgs::msg::Control::DISARMED;
    publish_control(msg_time, {});
    RCLCPP_INFO(get_logger(), "disarmed");
  }

  void ROVNode::start_rov(const rclcpp::Time &msg_time)
  {
    stop_mission();
    mode_ = orca_msgs::msg::Control::ROV;
    publish_control(msg_time, {});
    RCLCPP_INFO(get_logger(), "manual");
  }

  void ROVNode::start_hold_pressure(const rclcpp::Time &msg_time)
  {
    stop_mission();
    pressure_hold_pid_->set_target(pressure_);
    mode_ = orca_msgs::msg::Control::ROV_HOLD_PRESSURE;
    publish_control(msg_time, {});
    RCLCPP_INFO(get_logger(), "hold pressure at %g", pressure_);
  }

  void ROVNode::start_mission(const rclcpp::Time &msg_time, Mission mission, const geometry_msgs::msg::Pose &pose)
  {
    // Disarm to avoid conflicts with auv_node
    disarm(msg_time);

    // Wait for action server
    if (!mission_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "mission action server not responding, is auv_node running?");
      return;
    }

    auto goal_msg = MissionAction::Goal();
    switch (mission) {
      case Mission::KEEP_STATION:
        goal_msg.pose_targets = true;
        goal_msg.keep_station = true;
        RCLCPP_INFO(get_logger(), "keeping station at current pose");
        break;
      case Mission::GO_TO_POSE:
        goal_msg.pose_targets = true;
        goal_msg.poses.push_back(pose);
        RCLCPP_INFO(get_logger(), "go to pose");
        break;
      case Mission::RANDOM_MARKERS:
        goal_msg.random = true;
        RCLCPP_INFO(get_logger(), "visit all markers in a random order");
        break;
    }

    using namespace std::placeholders;
    auto send_goal_options = rclcpp_action::Client<MissionAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&ROVNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&ROVNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&ROVNode::result_callback, this, _1);

    // Send goal to auv_node
    auto goal_handle_future = mission_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // Don't change mode_, let caller do that
  void ROVNode::stop_mission()
  {
    if (auv_mode()) {
      mission_client_->async_cancel_all_goals();
      RCLCPP_INFO(get_logger(), "canceling all goals");
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
  auto node = std::make_shared<orca_base::ROVNode>();

  // Set logger level
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  (void) result;

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
