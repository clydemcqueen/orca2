#include "orca_base/base_node.hpp"

#include "cv_bridge/cv_bridge.h"

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
    (void) fcam_image_sub_;
    (void) fcam_info_sub_;
    (void) goal_sub_;
    (void) joy_sub_;
    (void) leak_sub_;
    (void) map_sub_;
    (void) obs_sub_;
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
    fcam_predicted_obs_pub_ = create_publisher<sensor_msgs::msg::Image>("fcam_predicted_obs", 1);
    esimated_path_pub_ = create_publisher<nav_msgs::msg::Path>("filtered_path", 1);
    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 1);
    target_path_pub_ = create_publisher<nav_msgs::msg::Path>("target_path", 1);
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);
    thrust_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("thrust_markers", 1);

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
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1, [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void
      { this->joy_cb_.call(msg); });
    map_sub_ = create_subscription<fiducial_vlam_msgs::msg::Map>(
      "fiducial_map", 1, [this](const fiducial_vlam_msgs::msg::Map::SharedPtr msg) -> void
      { this->map_cb_.call(msg); });

    using namespace std::placeholders;

    // Sync subscriptions
    fiducial_sync_.reset(new FiducialSync(FiducialPolicy(1), obs_sub_, fcam_pose_sub_));
    fiducial_sync_->registerCallback(std::bind(&BaseNode::fiducial_callback, this, _1, _2));
    obs_sub_.subscribe(this, "fiducial_observations"); // Uses default qos
    fcam_pose_sub_.subscribe(this, "fcam_f_map"); // Uses default qos

    // Other subscriptions
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

    // Parse URDF
    if (!parser_.parse()) {
      RCLCPP_ERROR(get_logger(), "can't parse URDF %s", orca_description::filename);
    } else {
      std::cout << "baro " << to_str_rpy(parser_.t_baro_base) << std::endl;
      std::cout << "fcam " << to_str_rpy(parser_.t_fcam_base) << std::endl;
      std::cout << "lcam " << to_str_rpy(parser_.t_lcam_base) << std::endl;
      std::cout << "rcam " << to_str_rpy(parser_.t_rcam_base) << std::endl;
      RCLCPP_INFO(get_logger(), "base_node ready");
    }
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
  void BaseNode::baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg)
  {
    pressure_ = msg->pressure;
    z_ = barometer_.pressure_to_depth(cxt_.model_, msg->pressure);
  }

  // New battery reading
  void BaseNode::battery_callback(const orca_msgs::msg::Battery::SharedPtr msg)
  {
    if (msg->low_battery) {
      RCLCPP_ERROR(get_logger(), "low battery (%g volts), disarming", msg->voltage);
      disarm(msg->header.stamp);
    }
  }

  void draw_text(cv::Mat &image, const std::string &s, const cv::Point2d &p, cv::Scalar color)
  {
    putText(image, s.c_str(), p, cv::FONT_HERSHEY_PLAIN, 1, color);
  }

  void BaseNode::write_status(cv::Mat &image)
  {
    cv::Point2d p{10, 20};
    std::stringstream ss;
    ss.precision(3);

    if (mode_ == orca_msgs::msg::Control::DISARMED) {
      ss << "DISARMED";
    } else if (mode_ == orca_msgs::msg::Control::ROV) {
      ss << "ARMED";
    } else if (mode_ == orca_msgs::msg::Control::ROV_HOLD_PRESSURE) {
      ss << "HOLD PRESSURE z=" << z_;
    } else if (mission_ != nullptr) {
      ss << "MISSION target m" << mission_->planner().target_marker_id();
      if (mission_->planner().in_recovery()) {
        ss << " RECOVERY move to m" << mission_->planner().recovery_marker_id();
      }
    } else {
      ss << "ERROR";
    }
    draw_text(image, ss.str(), p, CV_RGB(255, 0, 0));

    p.y += 20;
    ss.str("");

    ss << estimate_.fp.observations.size() << " observation(s)";
    draw_text(image, ss.str(), p, CV_RGB(0, 255, 0));
  }

  void draw_observations(cv::Mat &image, const std::vector<orca::Observation> &observations, cv::Scalar color)
  {
    for (const auto &obs : observations) {
      // Draw marker name
      cv::Point2d p = obs.c3;
      p.y += 30;
      std::stringstream ss;
      ss.precision(3);

      ss << "m" << obs.id << " d=" << obs.distance;
      draw_text(image, ss.str(), p, color);

      // Draw bounding box
      cv::line(image, obs.c0, obs.c1, color);
      cv::line(image, obs.c1, obs.c2, color);
      cv::line(image, obs.c2, obs.c3, color);
      cv::line(image, obs.c3, obs.c0, color);
    }
  }

  void BaseNode::fcam_image_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (fcam_predicted_obs_pub_->get_subscription_count() > 0) {
      CvFont font;
      cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.5, 0.5); // TODO how expensive is this?
      cv_bridge::CvImagePtr marked;
      marked = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      draw_observations(marked->image, plan_.observations, CV_RGB(255, 0, 0));
      draw_observations(marked->image, estimate_.fp.observations, CV_RGB(0, 255, 0));
      write_status(marked->image);

      fcam_predicted_obs_pub_->publish(*marked->toImageMsg());
    }
  }

  void BaseNode::fcam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // TODO make sure we have cam info before starting mission
    // std::cout << "got cam info" << std::endl;
    fcam_model_.fromCameraInfo(msg);
  }

  // Start a mission to move to a particular goal
  void BaseNode::goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (disarmed()) {
      FP goal;
      goal.pose.pose.from_msg(msg->pose);
      goal.pose.pose.z = cxt_.auv_z_target_;
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
      } else if (button_down(msg, joy_msg_, joy_button_auv_keep_station_)) {
        if (fp_ok(msg->header.stamp) && map_.ok()) {
          set_mode(msg->header.stamp, orca_msgs::msg::Control::AUV_KEEP_STATION);
        } else {
          RCLCPP_ERROR(get_logger(), "no odometry | no map | invalid filter, cannot keep station");
        }
      } else if (button_down(msg, joy_msg_, joy_button_auv_random_)) {
        if (fp_ok(msg->header.stamp) && map_.ok()) {
          set_mode(msg->header.stamp, orca_msgs::msg::Control::AUV_RANDOM);
        } else {
          RCLCPP_ERROR(get_logger(), "no odometry | no map | invalid filter, cannot run random mission");
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
      // TODO L/R controls are too sensitive
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

  void BaseNode::fiducial_callback(
    const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &obs_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &fcam_msg)
  {
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
    if (tf_pub_->get_subscription_count() > 0) {
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
    estimate_.from_msgs(*obs_msg, base_f_map);

    if (cxt_.sensor_loop_) {
      // Skip the first message -- the dt will be too large
      if (first) {
        return;
      }

      // Reject large dt
      double dt = (curr_time - prev_time).seconds();
      if (dt > 0.5) {
        return;
      }

      // Continue mission
      if (auv_mode()) {
        auv_advance(dt);
      }
    }
  }

  rclcpp_action::GoalResponse BaseNode::mission_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const orca_msgs::action::Mission::Goal> goal)
  {
    // Accept a mission if the sub is disarmed, we have odometry and we have a map
    if (disarmed() && fp_ok(now()) && map_.ok()) {
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
    set_mode(now(), goal_handle->get_goal()->mode, FP{}, goal_handle);
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
    // Slam in z TODO better way to do this?
    estimate_.fp.pose.pose.z = z_;
    estimate_.fp.pose.z_valid = true;

    // Publish local plan
    // TODO
//    if (count_subscribers(planned_path_pub_->get_topic_name()) > 0) {
//      planned_path_pub_->publish(mission_->planned_path());
//    }

    // Publish estimated path
    if (estimate_.fp.good_pose() && count_subscribers(esimated_path_pub_->get_topic_name()) > 0) {
      if (estimated_path_.poses.size() > cxt_.keep_poses_) {
        estimated_path_.poses.clear();
      }
      estimate_.add_to_path(estimated_path_);
      esimated_path_pub_->publish(estimated_path_);
    }

    // Advance plan and compute efforts
    Efforts efforts;
    if (mission_->advance(dt, plan_, estimate_, efforts)) {
      // Error for diagnostics (only useful if we have a good pose)
      Pose error = plan_.pose.pose.error(estimate_.fp.pose.pose);
      publish_control(estimate_.t, error, efforts);
    } else {
      // Mission is over, clean up
      mission_ = nullptr;
      plan_ = {};
      disarm(estimate_.t);
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
    // TODO
    //control_msg.odom_lag = (now() - odom_cb_.curr()).seconds();
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
  void BaseNode::set_mode(const rclcpp::Time &msg_time, uint8_t new_mode, const FP &goal,
                          const std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> &goal_handle)
  {
    using orca_msgs::msg::Control;

    // Stop, clean up old state, etc.
    disarm(msg_time);

    if (is_hold_pressure_mode(new_mode)) {

      RCLCPP_INFO(get_logger(), "hold pressure at %g", pressure_);
      pressure_hold_pid_->set_target(pressure_);

    } else if (is_auv_mode(new_mode)) {

      // Create planner, will build a global and local plan
      auto planner = std::make_shared<MissionPlanner>(get_logger(), cxt_, map_, parser_, fcam_model_);
      switch (new_mode) {
        case Control::AUV_KEEP_ORIGIN: {
          FP origin;
          origin.pose.pose.z = cxt_.auv_z_target_;
          planner->plan_target(origin, true);
          break;
        }
        case Control::AUV_SEQUENCE:
          planner->plan_wall_markers(false);
          break;
        case Control::AUV_RANDOM:
          planner->plan_wall_markers(true);
          break;
        case Control::AUV_GOAL:
          planner->plan_target(goal, true);
          break;
        case Control::AUV_KEEP_STATION:
        default:
          planner->plan_target(estimate_.fp, true);
          break;
      }

      // Publish global path for rviz
      if (count_subscribers(target_path_pub_->get_topic_name()) > 0) {
        target_path_pub_->publish(planner->global_path());
      }

      // Create mission, passing in the planner
      mission_ = std::make_shared<Mission>(get_logger(), cxt_, goal_handle, planner, estimate_);

      // Init estimated path
      estimated_path_.header.stamp = joy_cb_.curr();
      estimated_path_.header.frame_id = cxt_.map_frame_;
      estimated_path_.poses.clear();
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

//    if (auv_mode() && stability_ < 0.4) {
//      RCLCPP_ERROR(get_logger(), "excessive tilt during AUV operation, disarming");
//      disarm(spin_time);
//    }

    if (holding_pressure() && !baro_ok(spin_time)) {
      RCLCPP_ERROR(get_logger(), "lost barometer while holding pressure, disarming");
      disarm(spin_time);
    }

    if (auv_mode() && !baro_ok(spin_time)) {
      RCLCPP_ERROR(get_logger(), "lost barometer during AUV operation, disarming");
      disarm(spin_time);
    }

    if (!fp_ok(spin_time)) {
      // We lost observations, nuke estimate_
      estimate_ = FPStamped{};
    }

    // Auto-start mission
    if (!auv_mode() && is_auv_mode(cxt_.auto_start_) && !joy_ok(spin_time) && fp_ok(spin_time)) {
      RCLCPP_INFO(get_logger(), "auto-starting mission %d", cxt_.auto_start_);
      set_mode(spin_time, cxt_.auto_start_);
    }

    // Advance the mission
    if (!cxt_.sensor_loop_ && auv_mode()) {
      static std::chrono::duration<double> dt = SPIN_PERIOD;
      auv_advance(dt.count());
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
