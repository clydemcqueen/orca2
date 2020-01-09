#include "orca_base/base_node.hpp"

#include "orca_shared/pwm.hpp"

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

  //=============================================================================
  // BaseNode
  //=============================================================================

  BaseNode::BaseNode() : Node{"base_node"}, map_{get_logger(), cxt_}
  {
    // Suppress IDE warnings
    (void) baro_sub_;
    (void) battery_sub_;
    (void) joy_sub_;
    (void) leak_sub_;
    (void) map_sub_;
    (void) odom_sub_;
    (void) spin_timer_;

    // Get parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(BASE_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), BASE_NODE_ALL_PARAMS, validate_parameters)

    // ROV PID controller
    pressure_hold_pid_ = std::make_shared<pid::Controller>(false, cxt_.rov_pressure_pid_kp_, cxt_.rov_pressure_pid_ki_,
                                                           cxt_.rov_pressure_pid_kd_);

    // Publications
    control_pub_ = create_publisher<orca_msgs::msg::Control>("control", 1);
    thrust_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("thrust_markers", 1);
    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 1);
    filtered_path_pub_ = create_publisher<nav_msgs::msg::Path>("filtered_path", 1);

    // Monotonic subscriptions
    baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
      "barometer", 1, [this](const orca_msgs::msg::Barometer::SharedPtr msg) -> void
      { this->baro_cb_.call(msg); });
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1, [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void
      { this->joy_cb_.call(msg); });
    map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
      "fiducial_map", 1, [this](const fiducial_vlam_msgs::msg::Map::SharedPtr msg) -> void
      { this->map_cb_.call(msg); });
    obs_sub_ = create_subscription<fiducial_vlam_msgs::msg::Observations>(
      "fiducial_observations", 1, [this](const fiducial_vlam_msgs::msg::Observations::SharedPtr msg) -> void
      { this->obs_cb_.call(msg); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void
      { this->odom_cb_.call(msg); });

    // Other subscriptions
    using namespace std::placeholders;
    auto battery_cb = std::bind(&BaseNode::battery_callback, this, _1);
    battery_sub_ = create_subscription<orca_msgs::msg::Battery>("battery", 1, battery_cb);
    auto goal_cb = std::bind(&BaseNode::goal_callback, this, _1);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1, goal_cb);
    auto leak_cb = std::bind(&BaseNode::leak_callback, this, _1);
    leak_sub_ = create_subscription<orca_msgs::msg::Leak>("leak", 1, leak_cb);

    // Action server
    mission_server_ = rclcpp_action::create_server<orca_msgs::action::Mission>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "mission",
      std::bind(&BaseNode::mission_goal, this, _1, _2),
      std::bind(&BaseNode::mission_cancel, this, _1),
      std::bind(&BaseNode::mission_accepted, this, _1));

    // Loop will run at ~constant wall speed, switch to ros_timer when it exists
    spin_timer_ = create_wall_timer(SPIN_PERIOD, std::bind(&BaseNode::spin_once, this));

    RCLCPP_INFO(get_logger(), "base_node ready");
  }

  void BaseNode::validate_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_DEBUG, get_logger(), cxt_, n, t, d)
    BASE_NODE_ALL_PARAMS

    // Update model from new parameters
    cxt_.model_.fluid_density_ = cxt_.param_fluid_density_;
  }

  // New barometer reading
  void BaseNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg, bool first)
  {
    (void) first;

    pressure_ = msg->pressure;
  }

  // New battery reading
  void BaseNode::battery_callback(const orca_msgs::msg::Battery::SharedPtr msg)
  {
    if (msg->low_battery) {
      RCLCPP_ERROR(get_logger(), "low battery (%g volts), disarming", msg->voltage);
      disarm(msg->header.stamp);
    }
  }

  // Start a mission to move to a particular goal
  void BaseNode::goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (disarmed()) {
      Pose goal;
      goal.from_msg(msg->pose);
      goal.z = cxt_.auv_z_target_;
      set_mode(msg->header.stamp, orca_msgs::msg::Control::AUV_GOAL, goal);
    } else {
      RCLCPP_ERROR(get_logger(), "must be disarmed to start mission");
    }
  }

  // New input from the gamepad
  void BaseNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg, bool first)
  {
    if (!first) {
      // Arm/disarm
      if (button_down(msg, joy_msg_, joy_button_disarm_)) {
        RCLCPP_INFO(get_logger(), "disarmed");
        disarm(msg->header.stamp);
      } else if (button_down(msg, joy_msg_, joy_button_arm_)) {
        RCLCPP_INFO(get_logger(), "armed, manual");
        set_mode(msg->header.stamp, orca_msgs::msg::Control::ROV);
      }

      // If we're disarmed, ignore everything else
      if (mode_ == orca_msgs::msg::Control::DISARMED) {
        joy_msg_ = *msg;
        return;
      }

      // Mode
      if (button_down(msg, joy_msg_, joy_button_rov_)) {
        RCLCPP_INFO(get_logger(), "manual");
        set_mode(msg->header.stamp, orca_msgs::msg::Control::ROV);
      } else if (button_down(msg, joy_msg_, joy_button_rov_hold_pressure_)) {
        if (baro_ok(msg->header.stamp)) {
          set_mode(msg->header.stamp, orca_msgs::msg::Control::ROV_HOLD_PRESSURE);
        } else {
          RCLCPP_ERROR(get_logger(), "barometer not ready, cannot hold pressure");
        }
      } else if (button_down(msg, joy_msg_, joy_button_auv_keep_origin_)) {
        if (odom_ok(msg->header.stamp) && map_.ok()) {
          set_mode(msg->header.stamp, orca_msgs::msg::Control::AUV_KEEP_ORIGIN);
        } else {
          RCLCPP_ERROR(get_logger(), "no odometry | no map | invalid filter, cannot keep origin");
        }
      } else if (button_down(msg, joy_msg_, joy_button_auv_keep_station_)) {
        if (odom_ok(msg->header.stamp) && map_.ok()) {
          set_mode(msg->header.stamp, orca_msgs::msg::Control::AUV_KEEP_STATION);
        } else {
          RCLCPP_ERROR(get_logger(), "no odometry | no map | invalid filter, cannot keep station");
        }
      } else if (button_down(msg, joy_msg_, joy_button_move_to_marker_)) {
        if (obs_ok(msg->header.stamp) && observation_.id != orca::Observation::INVALID_ID) {
          set_mode(msg->header.stamp, orca_msgs::msg::Control::AUV_10);
        } else {
          RCLCPP_ERROR(get_logger(), "marker 0 not in view, cannot start move to marker mission");
        }
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
  void BaseNode::leak_callback(const orca_msgs::msg::Leak::SharedPtr msg)
  {
    if (msg->leak_detected) {
      RCLCPP_ERROR(get_logger(), "leak detected, disarming");
      disarm(msg->header.stamp);
    }
  }

  // New map available
  void BaseNode::map_callback(const fiducial_vlam_msgs::msg::Map::SharedPtr msg)
  {
    map_.set_vlam_map(msg);
  }

  // New odometry available
  void BaseNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, bool first)
  {
    // Save the pose
    filtered_odom_ = *msg;
    filtered_pose_.from_msg(*msg);

    // Compute a stability metric
    double roll, pitch, yaw;
    get_rpy(msg->pose.pose.orientation, roll, pitch, yaw);
    stability_ = std::min(clamp(std::cos(roll), 0.0, 1.0), clamp(std::cos(pitch), 0.0, 1.0));

    // Continue the mission
    // TODO check for !first
    if (auv_mode()) {
      auv_advance(odom_cb_.dt());
    }
  }

  // New marker observation available
  void BaseNode::obs_callback(fiducial_vlam_msgs::msg::Observations::SharedPtr msg, bool first)
  {
    // Save the observation
    observation_.from_msg(0, *msg);

    // Continue the mission
    if (!first && mtm_mode()) {
      mtm_advance(obs_cb_.dt());
    }
  }

  rclcpp_action::GoalResponse BaseNode::mission_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const orca_msgs::action::Mission::Goal> goal)
  {
    // Accept a mission if the sub is disarmed, we have odometry and we have a map
    if (disarmed() && odom_ok(now()) && map_.ok()) {
      RCLCPP_INFO(get_logger(), "mission %d accepted", goal->mode);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_WARN(get_logger(), "mission %d rejected", goal->mode);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse BaseNode::mission_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle)
  {
    // Always accept a cancellation
    RCLCPP_INFO(get_logger(), "cancel mission %d", goal_handle->get_goal()->mode);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void BaseNode::mission_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle)
  {
    // Start the mission
    RCLCPP_INFO(get_logger(), "execute mission %d", goal_handle->get_goal()->mode);
    set_mode(now(), goal_handle->get_goal()->mode, Pose{}, goal_handle);
  }

  void BaseNode::rov_advance(const rclcpp::Time &stamp)
  {
    double dt = joy_cb_.dt();

    Efforts efforts;
    efforts.set_forward(dead_band(joy_msg_.axes[joy_axis_forward_], cxt_.input_dead_band_) * cxt_.xy_gain_);
    efforts.set_strafe(dead_band(joy_msg_.axes[joy_axis_strafe_], cxt_.input_dead_band_) * cxt_.xy_gain_);
    efforts.set_yaw(dead_band(joy_msg_.axes[joy_axis_yaw_], cxt_.input_dead_band_) * cxt_.yaw_gain_);

    if (holding_pressure()) {
      efforts.set_vertical(
        Model::accel_to_effort_z(-pressure_hold_pid_->calc(pressure_, dt) + cxt_.model_.hover_accel_z()) * stability_);
    } else {
      efforts.set_vertical(dead_band(joy_msg_.axes[joy_axis_vertical_], cxt_.input_dead_band_) * cxt_.vertical_gain_);
    }

    Pose error;
    publish_control(stamp, error, efforts);
  }

  void BaseNode::auv_advance(double dt)
  {
    // Publish planned path for rviz
    if (count_subscribers(planned_path_pub_->get_topic_name()) > 0) {
      planned_path_pub_->publish(mission_->planned_path());
    }

    // Publish actual path for rviz
    if (full_pose(filtered_odom_) && count_subscribers(filtered_path_pub_->get_topic_name()) > 0) {
      if (filtered_path_.poses.size() > cxt_.keep_poses_) {
        filtered_path_.poses.clear();
      }
      filtered_pose_.add_to_path(filtered_path_);
      filtered_path_pub_->publish(filtered_path_);
    }

    // Advance plan and compute feedforward
    Pose plan;
    Acceleration u_bar;
    if (mission_->advance(dt, plan, filtered_odom_, u_bar)) {
      // Acceleration => effort
      Efforts efforts;
//      efforts.from_acceleration(filtered_pose_.pose.yaw, u_bar);
      efforts.from_acceleration(plan.yaw, u_bar);

      // Throttle back if AUV is unstable
      efforts.scale(stability_);

      // Compute error for diagnostics
      Pose error = plan.error(filtered_pose_.pose);

      publish_control(filtered_odom_.header.stamp, error, efforts);
    } else {
      // Mission is over, clean up
      mission_ = nullptr;
      disarm(filtered_odom_.header.stamp);
    }
  }

  void BaseNode::mtm_advance(double dt)
  {
    // Advance plan and compute feedforward
    if (mtm_segment_->advance(dt)) {
      // Calc acceleration
      Acceleration u_bar;
      mtm_controller_->calc(dt, mtm_segment_->plan(), observation_, mtm_segment_->ff(), u_bar);

      // Acceleration in the body frame => effort
      Efforts efforts;
      efforts.set_forward(Model::accel_to_effort_xy(u_bar.x));
      efforts.set_strafe(0);
      efforts.set_vertical(Model::accel_to_effort_xy(u_bar.z));
      efforts.set_yaw(Model::accel_to_effort_yaw(u_bar.yaw));

      // Error doesn't make sense here
      Pose error;

      publish_control(filtered_odom_.header.stamp, error, efforts);

    } else {
      // Mission is over, clean up
      RCLCPP_INFO(get_logger(), "mission completed");
      mtm_segment_ = nullptr;
      mtm_controller_ = nullptr;
      disarm(observation_.stamp);
    }
  }


  void BaseNode::all_stop(const rclcpp::Time &msg_time)
  {
    Pose error;
    Efforts efforts;
    publish_control(msg_time, error, efforts);
  }

  void BaseNode::publish_control(const rclcpp::Time &msg_time, const Pose &error, const Efforts &efforts)
  {
    // Combine joystick efforts to get thruster efforts.
    std::vector<double> thruster_efforts = {};
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
    error.to_msg(control_msg.error);
    efforts.to_msg(control_msg.efforts);
    control_msg.mode = mode_;
    control_msg.camera_tilt_pwm = tilt_to_pwm(tilt_);
    control_msg.brightness_pwm = brightness_to_pwm(brightness_);
    for (double thruster_effort : thruster_efforts) {
      control_msg.thruster_pwm.push_back(effort_to_pwm(thruster_effort));
    }
    control_msg.stability = stability_;
    control_msg.odom_lag = (now() - odom_cb_.curr()).seconds();
    control_pub_->publish(control_msg);

    // Publish rviz thrust markers
    if (count_subscribers(thrust_marker_pub_->get_topic_name()) > 0) {
      visualization_msgs::msg::MarkerArray markers_msg;
      for (unsigned long i = 0; i < thruster_efforts.size(); ++i) {
        int32_t action =
          thruster_efforts[i] == 0.0 ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
        double scale = (THRUSTERS[i].ccw ? -thruster_efforts[i] : thruster_efforts[i]) / 5.0;
        double offset = scale > 0 ? -0.1 : 0.1;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = THRUSTERS[i].frame_id;
        marker.header.stamp = msg_time;
        marker.ns = "thruster";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = action;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = offset;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.7071068;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.7071068;
        marker.scale.x = scale;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markers_msg.markers.push_back(marker);
      }
      thrust_marker_pub_->publish(markers_msg);
    }
  }

  void BaseNode::disarm(const rclcpp::Time &msg_time)
  {
    if (mode_ != orca_msgs::msg::Control::DISARMED) {
      // Stop all thrusters
      all_stop(msg_time);

      // Abort an active mission
      if (mission_) {
        mission_->abort();
        mission_ = nullptr;
      }

      // Set mode
      mode_ = orca_msgs::msg::Control::DISARMED;
    }
  }

  // Change operation mode
  void BaseNode::set_mode(const rclcpp::Time &msg_time, uint8_t new_mode, const Pose &goal,
                          const std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> &goal_handle)
  {
    using orca_msgs::msg::Control;

    // Stop, clean up old state, etc.
    disarm(msg_time);

    if (is_hold_pressure_mode(new_mode)) {

      RCLCPP_INFO(get_logger(), "hold pressure at %g", pressure_);
      pressure_hold_pid_->set_target(pressure_);

    } else if (is_auv_mode(new_mode)) {

      std::shared_ptr<PlannerBase> planner;
      switch (new_mode) {
        case Control::AUV_KEEP_ORIGIN: {
          Pose origin;
          origin.z = cxt_.auv_z_target_;
          planner = std::make_shared<TargetPlanner>(get_logger(), cxt_, map_, origin, true);
          break;
        }
        case Control::AUV_SEQUENCE:
          planner = std::make_shared<DownSequencePlanner>(get_logger(), cxt_, map_, false);
          break;
        case Control::AUV_RANDOM:
          planner = std::make_shared<DownSequencePlanner>(get_logger(), cxt_, map_, true);
          break;
        case Control::AUV_GOAL:
          planner = std::make_shared<TargetPlanner>(get_logger(), cxt_, map_, goal, false);
          break;
        case Control::AUV_KEEP_STATION:
        default:
          planner = std::make_shared<TargetPlanner>(get_logger(), cxt_, map_, filtered_pose_.pose, true);
          break;
      }

      mission_ = std::make_shared<Mission>(get_logger(), cxt_, goal_handle, planner, filtered_pose_);

      // Init filtered_path
      filtered_path_.header.stamp = joy_cb_.curr();
      filtered_path_.header.frame_id = cxt_.map_frame_;
      filtered_path_.poses.clear();

    } else if (is_mtm_mode(new_mode)) {

      RCLCPP_INFO(get_logger(), "move to marker");
      orca::Observation goal_observation;
      goal_observation.id = 0;
      goal_observation.distance = 1.5;
      goal_observation.yaw = 0;
      mtm_segment_ = std::make_shared<MoveToMarkerSegment>(cxt_, 0, observation_, goal_observation);
      mtm_controller_ = std::make_shared<MoveToMarkerController>(cxt_);

    }

    // Set the new mode
    mode_ = new_mode;
  }

  void BaseNode::spin_once()
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

    if (auv_mode() && !odom_ok(spin_time)) {
      RCLCPP_ERROR(get_logger(), "lost odometry during AUV operation, disarming");
      disarm(spin_time);
    }

    if (auv_mode() && stability_ < 0.4) {
      RCLCPP_ERROR(get_logger(), "excessive tilt during AUV operation, disarming");
      disarm(spin_time);
    }

    if (holding_pressure() && !baro_ok(spin_time)) {
      RCLCPP_ERROR(get_logger(), "lost barometer while holding pressure, disarming");
      disarm(spin_time);
    }

    if (mtm_mode() && !obs_ok(spin_time)) {
      RCLCPP_ERROR(get_logger(), "lost observations while moving to marker, disarming");
      disarm(spin_time);
    }

    // Auto-start mission
    if (!auv_mode() && is_auv_mode(cxt_.auto_start_) && !joy_ok(spin_time) && odom_ok(spin_time)) {
      RCLCPP_INFO(get_logger(), "auto-starting mission %d", cxt_.auto_start_);
      set_mode(spin_time, cxt_.auto_start_);
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
  auto node = std::make_shared<orca_base::BaseNode>();

  // Set logger level
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
