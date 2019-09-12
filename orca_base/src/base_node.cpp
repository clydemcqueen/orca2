#include "orca_base/base_node.hpp"
#include "orca_base/pwm.hpp"

namespace orca_base
{

  //=============================================================================
  // Constants
  //=============================================================================

  const double Z_HOLD_MAX = -0.05;  // Highest z hold
  const double Z_HOLD_MIN = -50;    // Lowest z hold

  //=============================================================================
  // BaseNode
  //=============================================================================

  BaseNode::BaseNode() : Node{"base_node"}
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
    rov_z_pid_ = std::make_shared<pid::Controller>(false, cxt_.rov_z_pid_kp_, cxt_.rov_z_pid_ki_, cxt_.rov_z_pid_kd_);

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
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "filtered_odom", 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void
      { this->odom_cb_.call(msg); });

    // Other subscriptions
    using std::placeholders::_1;
    auto battery_cb = std::bind(&BaseNode::battery_callback, this, _1);
    battery_sub_ = create_subscription<orca_msgs::msg::Battery>("battery", 1, battery_cb);
    auto leak_cb = std::bind(&BaseNode::leak_callback, this, _1);
    leak_sub_ = create_subscription<orca_msgs::msg::Leak>("leak", 1, leak_cb);

    // Loop will run at ~constant wall speed, switch to ros_timer when it exists
    spin_timer_ = create_wall_timer(SPIN_PERIOD, std::bind(&BaseNode::spin_once, this));

    RCLCPP_INFO(get_logger(), "base_node ready");
  }

  void BaseNode::validate_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    BASE_NODE_ALL_PARAMS
  }

  // New barometer reading
  void BaseNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg, bool first)
  {
    if (first) {
      z_initial_ = -msg->depth;
      z_ = 0;
      RCLCPP_INFO(get_logger(), "barometer adjustment %g", z_initial_);
    } else {
      z_ = -msg->depth - z_initial_;
    }
  }

  // New battery reading
  void BaseNode::battery_callback(const orca_msgs::msg::Battery::SharedPtr msg)
  {
    if (msg->low_battery) {
      RCLCPP_ERROR(get_logger(), "low battery (%g volts), disarming", msg->voltage);
      set_mode(msg->header.stamp, orca_msgs::msg::Control::DISARMED);
    }
  }

  // New input from the gamepad
  void BaseNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg, bool first)
  {
    if (!first) {
      // Arm/disarm
      if (button_down(msg, joy_msg_, joy_button_disarm_)) {
        RCLCPP_INFO(get_logger(), "disarmed");
        set_mode(msg->header.stamp, orca_msgs::msg::Control::DISARMED);
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
      } else if (button_down(msg, joy_msg_, joy_button_rov_hold_z_)) {
        if (baro_ok(msg->header.stamp)) {
          set_mode(msg->header.stamp, orca_msgs::msg::Control::ROV_HOLD_Z);
        } else {
          RCLCPP_ERROR(get_logger(), "barometer not ready, cannot hold z");
        }
      } else if (button_down(msg, joy_msg_, joy_button_auv_keep_station_)) {
        if (odom_ok(msg->header.stamp) && map_cb_.receiving()) {
          set_mode(msg->header.stamp, orca_msgs::msg::Control::AUV_KEEP_STATION);
        } else {
          RCLCPP_ERROR(get_logger(), "no odometry and/or no map, cannot keep station");
        }
      } else if (button_down(msg, joy_msg_, joy_button_auv_mission_4_)) {
        if (odom_ok(msg->header.stamp) && map_cb_.receiving()) {
          set_mode(msg->header.stamp, orca_msgs::msg::Control::AUV_4);
        } else {
          RCLCPP_ERROR(get_logger(), "no odometry and/or no map, cannot start mission 4");
        }
      } else if (button_down(msg, joy_msg_, joy_button_auv_mission_5_)) {
        if (odom_ok(msg->header.stamp) && map_cb_.receiving()) {
          set_mode(msg->header.stamp, orca_msgs::msg::Control::AUV_5);
        } else {
          RCLCPP_ERROR(get_logger(), "no odometry and/or no map, cannot start mission 5");
        }
      }

      // Z trim
      if (holding_z() && trim_down(msg, joy_msg_, joy_axis_z_trim_)) {
        rov_z_pid_->set_target(clamp(
          msg->axes[joy_axis_z_trim_] > 0 ? rov_z_pid_->target() + cxt_.inc_z_ : rov_z_pid_->target() - cxt_.inc_z_,
          Z_HOLD_MIN, Z_HOLD_MAX));
        RCLCPP_INFO(get_logger(), "hold z at %g", rov_z_pid_->target());
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
      set_mode(msg->header.stamp, orca_msgs::msg::Control::DISARMED);
    }
  }

  // New map available
  void BaseNode::map_callback(const fiducial_vlam_msgs::msg::Map::SharedPtr msg)
  {
    map_ = *msg;
  }

  // New pose estimate available
  void BaseNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, bool first)
  {
    filtered_pose_.from_msg(*msg);
    odom_lag_ = (now() - odom_cb_.curr()).seconds();

    // Compute a stability metric
    // TODO
    // stability_ = std::min(clamp(std::cos(roll), 0.0, 1.0), clamp(std::cos(pitch), 0.0, 1.0));

    if (auv_mode()) {
      auv_advance(msg->header.stamp, odom_cb_.dt());
    }
  }

  void BaseNode::rov_advance(const rclcpp::Time &stamp)
  {
    double dt = joy_cb_.dt();

    Efforts efforts;
    efforts.set_forward(dead_band(joy_msg_.axes[joy_axis_forward_], cxt_.input_dead_band_) * cxt_.xy_gain_);
    efforts.set_strafe(dead_band(joy_msg_.axes[joy_axis_strafe_], cxt_.input_dead_band_) * cxt_.xy_gain_);
    efforts.set_yaw(dead_band(joy_msg_.axes[joy_axis_yaw_], cxt_.input_dead_band_) * cxt_.yaw_gain_);

    if (holding_z()) {
      efforts.set_vertical(accel_to_effort_z(rov_z_pid_->calc(z_, dt) + HOVER_ACCEL_Z) * stability_);
    } else {
      efforts.set_vertical(dead_band(joy_msg_.axes[joy_axis_vertical_], cxt_.input_dead_band_) * cxt_.vertical_gain_);
    }

    Pose error;
    publish_control(stamp, error, efforts);
  }

  void BaseNode::auv_advance(const rclcpp::Time &msg_time, double dt)
  {
    // Publish path for rviz
    if (count_subscribers(filtered_path_pub_->get_topic_name()) > 0) {
      if (filtered_path_.poses.size() > cxt_.keep_poses_) {
        filtered_path_.poses.clear();
      }
      filtered_pose_.add_to_path(filtered_path_);
      filtered_path_pub_->publish(filtered_path_);
    }

    // Advance plan and compute feedforward
    Pose plan;
    Acceleration ff;
    if (mission_->advance(dt, plan, ff)) {
      // Compute error
      Pose error = plan.error(filtered_pose_.pose);

      // Compute acceleration due to error
      Acceleration u_bar;
      controller_->calc(cxt_, dt, plan, filtered_pose_.pose, ff, u_bar);

      // Acceleration => effort
      Efforts efforts;
      efforts.from_acceleration(u_bar, filtered_pose_.pose.yaw);

      // Throttle back if AUV is unstable
      efforts.scale(stability_);

      publish_control(msg_time, error, efforts);
    } else {
      // Stop mission
      set_mode(msg_time, orca_msgs::msg::Control::DISARMED);
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
    control_msg.odom_lag = odom_lag_;
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

  // Change operation mode
  void BaseNode::set_mode(const rclcpp::Time &msg_time, uint8_t new_mode)
  {
    // Stop all thrusters
    all_stop(msg_time);

    if (is_z_hold_mode(new_mode)) {
      rov_z_pid_->set_target(z_);
      RCLCPP_INFO(get_logger(), "hold z at %g", rov_z_pid_->target());
    }

    if (is_auv_mode(new_mode)) {
      std::shared_ptr<BasePlanner> planner;
      switch (new_mode) {
        case orca_msgs::msg::Control::AUV_4:
          RCLCPP_INFO(get_logger(), "creating a random plan for a forward-facing camera...");
          planner = std::make_shared<ForwardRandomPlanner>();
          break;
        case orca_msgs::msg::Control::AUV_5:
          RCLCPP_INFO(get_logger(), "creating a random plan for a down-facing camera...");
          planner = std::make_shared<DownRandomPlanner>();
          break;
        default:
//          RCLCPP_INFO(get_logger(), "keeping station at (%g, %g, %g), %g...",
//                      filtered_pose_.pose.x, filtered_pose_.pose.y, filtered_pose_.pose.z, filtered_pose_.pose.yaw);
//          planner = std::make_shared<KeepStationPlanner>();
          RCLCPP_INFO(get_logger(), "keeping station at (0, 0, %g), 0...", cxt_.auv_z_target_);
          planner = std::make_shared<OriginPlanner>();
          break;
      }
      mission_ = std::make_shared<Mission>(get_logger(), planner, cxt_, map_, filtered_pose_);
      switch (cxt_.auv_controller_) {
        case 1:
          controller_ = std::make_shared<DeadzoneController>(cxt_);
          break;
        case 2:
          controller_ = std::make_shared<JerkController>(cxt_);
          break;
        case 3:
          controller_ = std::make_shared<BestController>(cxt_);
          break;
        default:
          controller_ = std::make_shared<BaseController>(cxt_);
          break;
      }

      // Publish path for rviz
      if (count_subscribers(planned_path_pub_->get_topic_name()) > 0) {
        planned_path_pub_->publish(mission_->planned_path());
      }

      // Init filtered_path
      filtered_path_.header.stamp = joy_cb_.curr();
      filtered_path_.header.frame_id = cxt_.map_frame_;
      filtered_path_.poses.clear();
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
      set_mode(spin_time, orca_msgs::msg::Control::DISARMED);
    }

    if (auv_mode() && !odom_ok(spin_time)) {
      RCLCPP_ERROR(get_logger(), "lost odometry during AUV operation, disarming");
      set_mode(spin_time, orca_msgs::msg::Control::DISARMED);
    }

    if (auv_mode() && stability_ < 0.4) {
      RCLCPP_ERROR(get_logger(), "excessive tilt during AUV operation, disarming");
      set_mode(spin_time, orca_msgs::msg::Control::DISARMED);
    }

    if (holding_z() && !baro_ok(spin_time)) {
      RCLCPP_ERROR(get_logger(), "lost barometer while holding z, disarming");
      set_mode(spin_time, orca_msgs::msg::Control::DISARMED);
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

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
