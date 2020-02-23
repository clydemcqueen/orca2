#include "orca_base/base_node.hpp"

#include <iomanip>

#include "cv_bridge/cv_bridge.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

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
    cxt_.model_.fluid_density_ = cxt_.fluid_density_;
    cxt_.model_.drag_coef_f_ = cxt_.drag_coef_f_;
    cxt_.model_.drag_coef_s_ = cxt_.drag_coef_s_;
    cxt_.model_.drag_coef_z_ = cxt_.drag_coef_z_;
    cxt_.model_.drag_coef_tether_ = cxt_.drag_coef_tether_;
    cxt_.model_.drag_partial_const_yaw_ = cxt_.drag_partial_const_yaw_;

    // Update timeouts
    baro_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_baro_ms_)};
    fp_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_fp_ms_)};
    joy_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_joy_ms_)};
    spin_period_ = std::chrono::milliseconds{cxt_.timer_period_ms_};

    // [Re-]start loop
    // Loop will run at ~constant wall speed, switch to ros_timer when it exists
    spin_timer_ = create_wall_timer(spin_period_, std::bind(&BaseNode::spin_once, this));
  }

  bool BaseNode::holding_pressure()
  { return is_hold_pressure_mode(mode_); }

  bool BaseNode::rov_mode()
  { return is_rov_mode(mode_); }

  bool BaseNode::auv_mode()
  { return is_auv_mode(mode_); }

  bool BaseNode::baro_ok(const rclcpp::Time &t)
  { return baro_cb_.receiving() && t - baro_cb_.prev() < baro_timeout_; }

  bool BaseNode::joy_ok(const rclcpp::Time &t)
  { return joy_cb_.receiving() && t - joy_cb_.prev() < joy_timeout_; }

  bool BaseNode::fp_ok(const rclcpp::Time &t)
  { return monotonic::valid(estimate_.t) && t - estimate_.t < fp_timeout_; }

  bool BaseNode::cam_info_ok()
  { return fcam_info_cb_.receiving(); }


  bool BaseNode::ready_to_start_mission()
  {
    // -- not currently in a mission
    // -- have a map
    // -- have camera info
    // -- have a good pose or a good observation
    return !auv_mode() && map_.ok() && cam_info_ok() && fp_ok(now()) &&
           (estimate_.fp.good_pose(cxt_.good_pose_dist_) || estimate_.fp.closest_obs() < cxt_.good_obs_dist_);
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
    ss << std::fixed << std::setprecision(2);

    if (mode_ == orca_msgs::msg::Control::DISARMED) {
      ss << "DISARMED";
    } else if (mode_ == orca_msgs::msg::Control::ROV) {
      ss << "ARMED";
    } else if (mode_ == orca_msgs::msg::Control::ROV_HOLD_PRESSURE) {
      ss << "HOLD PRESSURE z=" << z_;
    } else if (mission_) {
      ss << "MISSION target m" << mission_->status().target_marker_id;
      if (mission_->status().planner == orca_msgs::msg::Control::PLAN_RECOVERY_MTM) {
        ss << " RECOVERY move to m" << mission_->status().target_marker_id;
      } else {
        ss << " plan z=" << plan_.fp.pose.pose.z;
      }
    } else {
      ss << "ERROR";
    }

    draw_text(image, ss.str(), p, CV_RGB(255, 0, 0));

    p.y += 20;
    ss.str("");

    ss << estimate_.fp.observations.size() << " observation(s)";
    ss << " estimate z=" << z_;

    draw_text(image, ss.str(), p, CV_RGB(0, 255, 0));
  }

  void draw_observations(cv::Mat &image, const std::vector<Observation> &observations, cv::Scalar color)
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

  void BaseNode::fcam_image_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (fcam_predicted_obs_pub_->get_subscription_count() > 0) {
      cv_bridge::CvImagePtr marked;
      marked = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      draw_observations(marked->image, plan_.fp.observations, CV_RGB(255, 0, 0));
      draw_observations(marked->image, estimate_.fp.observations, CV_RGB(0, 255, 0));
      write_status(marked->image);

      fcam_predicted_obs_pub_->publish(*marked->toImageMsg());
    }
  }

  void BaseNode::fcam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fcam_model_.fromCameraInfo(msg);
  }

  // Start a mission to move to a particular goal -- called by rviz2
  void BaseNode::goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (ready_to_start_mission()) {
      RCLCPP_INFO(get_logger(), "goal accepted");
      msg->pose.position.z = cxt_.auv_z_target_;
      start_mission(msg->header.stamp, true, {}, {msg->pose});
    } else {
      RCLCPP_ERROR(get_logger(), "cannot start mission");
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
        start_rov(msg->header.stamp);
      }

      // If we're disarmed, ignore everything else
      if (mode_ == orca_msgs::msg::Control::DISARMED) {
        joy_msg_ = *msg;
        return;
      }

      // Mode
      if (button_down(msg, joy_msg_, joy_button_rov_)) {
        RCLCPP_INFO(get_logger(), "manual");
        start_rov(msg->header.stamp);
      } else if (button_down(msg, joy_msg_, joy_button_rov_hold_pressure_)) {
        if (baro_ok(msg->header.stamp)) {
          start_hold_pressure(msg->header.stamp);
        } else {
          RCLCPP_ERROR(get_logger(), "barometer not ready, cannot hold pressure");
        }
      } else if (button_down(msg, joy_msg_, joy_button_auv_keep_station_)) {
        // Must be ready to start a mission && must have a good pose
        if (ready_to_start_mission() && estimate_.fp.good_pose(cxt_.good_pose_dist_)) {
          start_mission(msg->header.stamp, true, {}, {}, false, false, true);
        } else {
          RCLCPP_ERROR(get_logger(), "cannot start mission");
        }
      } else if (button_down(msg, joy_msg_, joy_button_auv_random_)) {
        if (ready_to_start_mission()) {
          // Pass in zero markers, the global planner will plan a mission with all of the markers in the map
          start_mission(msg->header.stamp);
        } else {
          RCLCPP_ERROR(get_logger(), "cannot start mission");
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

  // Get a synchronized set of messages from vlam: pose from SolvePnP and raw marker observations
  // Built for the non-filter case TODO also handle the filter case
  void BaseNode::fiducial_callback(
    const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &obs_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &fcam_msg)
  {
    // Ignore until we have a good map
    if (!map_.ok()) {
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
    estimate_.from_msgs(*obs_msg, base_f_map, z_, map_.marker_length(), cxt_.fcam_hfov_, cxt_.fcam_hres_);

    if (cxt_.sensor_loop_ && auv_mode()) {
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

  rclcpp_action::GoalResponse BaseNode::mission_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const orca_msgs::action::Mission::Goal> goal)
  {
    using orca_msgs::msg::Control;

    if (ready_to_start_mission()) {
      RCLCPP_INFO(get_logger(), "mission accepted");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_ERROR(get_logger(), "mission rejected");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse BaseNode::mission_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle)
  {
    // Always accept a cancellation
    RCLCPP_INFO(get_logger(), "cancel mission");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void BaseNode::mission_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle)
  {
    // Start the mission
    RCLCPP_INFO(get_logger(), "execute mission");
    auto action = goal_handle->get_goal();
    start_mission(now(), action->pose_targets, action->marker_ids, action->poses, action->random, action->repeat,
                  action->keep_station, goal_handle);
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

    publish_control(stamp, efforts);
  }

  void BaseNode::auv_advance(const rclcpp::Time &msg_time, const rclcpp::Duration &d)
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
    if (mission_->advance(d, plan_, estimate_, efforts)) {
      // Publish control message
      publish_control(msg_time, efforts);

      if (mission_->status().planner == orca_msgs::msg::Control::PLAN_LOCAL &&
          count_subscribers(planned_pose_pub_->get_topic_name()) > 0) {
        // Publish planned pose for visualization
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = cxt_.map_frame_;
        pose_msg.pose = plan_.fp.pose.pose.to_msg();
        planned_pose_pub_->publish(pose_msg);
      }
    } else {
      // Mission is over, clean up
      mission_ = nullptr;
      plan_ = {};
      disarm(msg_time);
    }
  }

  void BaseNode::all_stop(const rclcpp::Time &msg_time)
  {
    publish_control(msg_time, {});
  }

  void BaseNode::publish_control(const rclcpp::Time &msg_time, const orca::Efforts &efforts)
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
    control_msg.estimate_pose = estimate_.fp.pose.pose.to_msg();
    control_msg.efforts = efforts.to_msg();
    control_msg.stability = stability_;
    control_msg.odom_lag = (now() - msg_time).seconds();
    control_msg.mode = mode_;
    if (mission_) {
      // TODO control_msg.plan_pose
      // TODO control_msg.plan_twist
      control_msg.targets_total = mission_->status().targets_total;
      control_msg.target_idx = mission_->status().target_idx;
      control_msg.target_marker_id = mission_->status().target_marker_id;
      control_msg.planner = mission_->status().planner;
      control_msg.segments_total = mission_->status().segments_total;
      control_msg.segment_idx = mission_->status().segment_idx;
      control_msg.segment_info = mission_->status().segment_info;
    } else {
      control_msg.plan_pose = geometry_msgs::msg::Pose{};
      control_msg.plan_twist = geometry_msgs::msg::Twist{};
      control_msg.targets_total = 0;
      control_msg.target_idx = 0;
      control_msg.target_marker_id = 0;
      control_msg.planner = orca_msgs::msg::Control::PLAN_NONE;
      control_msg.segments_total = 0;
      control_msg.segment_idx = 0;
      control_msg.segment_info = "";
    }
    control_msg.camera_tilt_pwm = tilt_to_pwm(tilt_);
    control_msg.brightness_pwm = brightness_to_pwm(brightness_);
    for (double thruster_effort : thruster_efforts) {
      control_msg.thruster_pwm.push_back(effort_to_pwm(thruster_effort));
    }
    control_pub_->publish(control_msg);
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

  void BaseNode::start_rov(const rclcpp::Time &msg_time)
  {
    // Stop, clean up old state, etc.
    disarm(msg_time);

    // Set mode
    mode_ = orca_msgs::msg::Control::ROV;
  }

  void BaseNode::start_hold_pressure(const rclcpp::Time &msg_time)
  {
    // Stop, clean up old state, etc.
    disarm(msg_time);

    RCLCPP_INFO(get_logger(), "hold pressure at %g", pressure_);
    pressure_hold_pid_->set_target(pressure_);

    // Set mode
    mode_ = orca_msgs::msg::Control::ROV_HOLD_PRESSURE;
  }

  // Start a mission. There are lots of options! Here are some default cases:
  //
  // If !pose_targets, then look to markers for a list of markers.
  // If markers.empty(), then use all markers in the map
  //
  // If pose_targets, then look to poses for a list of poses.
  // If poses.empty(), then try to provide the current pose.
  // This only works if we have a good pose. It's only useful if keep_station is true.
  void BaseNode::start_mission(const rclcpp::Time &msg_time, bool pose_targets, const std::vector<int> &markers,
                               std::vector<geometry_msgs::msg::Pose> poses,
                               bool random, bool repeat, bool keep_station,
                               const std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> &goal_handle)
  {
    // Stop, clean up old state, etc.
    disarm(msg_time);

    RCLCPP_INFO(get_logger(), "start mission, pose_targets=%d, %d marker(s), %d pose(s), random=%d, repeat=%d, keep=%d",
                pose_targets, markers.size(), poses.size(), random, repeat, keep_station);

    if (pose_targets && poses.empty() && keep_station && estimate_.fp.good_pose(cxt_.good_pose_dist_)) {
      RCLCPP_INFO(get_logger(), "keeping station at current pose");
      poses.push_back(estimate_.fp.pose.pose.to_msg());
    }

    // Create planner, will build a global and local plan
    auto planner = pose_targets ?
                   GlobalPlanner::plan_poses(get_logger(), cxt_, map_, parser_, fcam_model_, poses,
                                             random, repeat, keep_station) :
                   GlobalPlanner::plan_markers(get_logger(), cxt_, map_, parser_, fcam_model_, markers,
                                               random, repeat, keep_station);

    if (!planner) {
      // Failed to build a global plan, abort mission
      if (goal_handle) {
        auto result = std::make_shared<orca_msgs::action::Mission::Result>();
        result->targets_total = 0;
        result->targets_completed = 0;
        goal_handle->abort(result);
      }

      // Leave in disarmed state
      return;
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

    // Set mode
    mode_ = orca_msgs::msg::Control::AUV;
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

    // Mission loop is either timer-driven or sensor driven
    // Note: control_msg.odom_lag will always be 0
    if (!cxt_.sensor_loop_ && auv_mode()) {
      auv_advance(now(), spin_period_);
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
