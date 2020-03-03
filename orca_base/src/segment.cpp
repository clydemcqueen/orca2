#include "orca_base/segment.hpp"

#include <iomanip>

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/util.hpp"

using namespace orca;

namespace orca_base
{

  //=====================================================================================
  // SegmentBase
  //=====================================================================================

  SegmentBase::SegmentBase(AUVContext cxt, uint8_t type) :
    cxt_{std::move(cxt)}, type_{type}
  {
  }

  std::string SegmentBase::type_name()
  {
    if (type_ == orca_msgs::msg::Control::PAUSE) {
      return "pause";
    } else if (type_ == orca_msgs::msg::Control::POSE_VERTICAL) {
      return "pose_vertical";
    } else if (type_ == orca_msgs::msg::Control::POSE_ROTATE) {
      return "pose_rotate";
    } else if (type_ == orca_msgs::msg::Control::POSE_LINE) {
      return "pose_line";
    } else if (type_ == orca_msgs::msg::Control::POSE_COMBO) {
      return "pose_combo";
    } else if (type_ == orca_msgs::msg::Control::OBS_RTM) {
      return "obs_rtm";
    } else if (type_ == orca_msgs::msg::Control::OBS_MTM) {
      return "obs_mtm";
    } else if (type_ == orca_msgs::msg::Control::POSE_LINE) {
      return "no_segment";
    }
  }

  //=====================================================================================
  // PoseSegmentBase
  //=====================================================================================

  PoseSegmentBase::PoseSegmentBase(const AUVContext &cxt, uint8_t type, FPStamped start, FP goal) :
    SegmentBase{cxt, type},
    plan_{std::move(start)},
    goal_{std::move(goal)}
  {
    // Default ff includes acceleration to counteract buoyancy
    ff_ = Acceleration{0, 0, cxt.model_.hover_accel_z(), 0};
  }

  //=====================================================================================
  // ObservationSegmentBase
  //=====================================================================================

  ObservationSegmentBase::ObservationSegmentBase(const AUVContext &cxt, uint8_t type, ObservationStamped start,
                                                 Observation goal) :
    SegmentBase{cxt, type},
    plan_{std::move(start)},
    goal_{std::move(goal)}
  {
    // Default ff includes acceleration to counteract buoyancy
    ff_ = AccelerationBody{0, 0, cxt.model_.hover_accel_z(), 0};
  }

  //=====================================================================================
  // Pause
  //=====================================================================================

  Pause::Pause(const AUVContext &cxt, const FPStamped &start, const rclcpp::Duration &pause_duration) :
    PoseSegmentBase{cxt, orca_msgs::msg::Control::PAUSE, start, start.fp}, pause_duration_{pause_duration}
  {}

  std::string Pause::to_str()
  {
    std::stringstream ss;
    ss << "pause for " << pause_duration_.seconds() << " seconds";
    return ss.str();
  }

  bool Pause::advance(const rclcpp::Duration &d)
  {
    // Update plan
    plan_.t = plan_.t + d;

    // Count down time remaining
    pause_duration_ = pause_duration_ - d;

    return pause_duration_.nanoseconds() > 0;
  }

  //=====================================================================================
  // TrapVelo
  //=====================================================================================

  // Plan a 3-phase trapezoidal velocity motion constrained by acceleration and max_velocity.
  //    phase 1: accelerate
  //    phase 2: run at constant velocity
  //    phase 3: decelerate and stop
  //
  // If the distance is short skip the run phase

  void plan_trap_velo(const double accel, const double max_velo, const double distance,
                      const rclcpp::Time &start, rclcpp::Time &run, rclcpp::Time &decel, rclcpp::Time &stop)
  {
    auto ramp_seconds = max_velo / accel;
    auto ramp_distance = accel * ramp_seconds * ramp_seconds / 2;

    double run_seconds;
    if (2 * ramp_distance < distance) {
      run_seconds = (distance - 2 * ramp_distance) / max_velo;
    } else {
      // Distance too short, will not hit max_velo
      // Note that distance = 2 * ramp_distance
      ramp_seconds = sqrt(distance / accel);
      run_seconds = 0;
    }

    run = start + rclcpp::Duration::from_seconds(ramp_seconds);
    decel = run + rclcpp::Duration::from_seconds(run_seconds);
    stop = decel + rclcpp::Duration::from_seconds(ramp_seconds);
  }

  TrapVelo::TrapVelo(const AUVContext &cxt, uint8_t type, const FPStamped &start, const FP &goal) :
    PoseSegmentBase{cxt, type, start, goal}, angle_to_goal_{0}, start_{start.t}
  {
    // Distance is always >= 0
    double distance_xy = plan_.fp.distance_xy(goal_);
    double distance_z = plan_.fp.distance_z(goal_);
    double distance_yaw = plan_.fp.distance_yaw(goal_);

    if (distance_xy > 0) {
      // Plan xy motion, start phase 1
      angle_to_goal_ = atan2(goal_.pose.pose.y - plan_.fp.pose.pose.y, goal_.pose.pose.x - plan_.fp.pose.pose.x);
      plan_trap_velo(cxt_.auv_xy_accel_, cxt_.auv_xy_velo_, distance_xy, plan_.t, xy_run_, xy_decel_, xy_stop_);
      initial_accel_.x = cos(angle_to_goal_) * cxt_.auv_xy_accel_;
      initial_accel_.y = sin(angle_to_goal_) * cxt_.auv_xy_accel_;
    } else {
      xy_run_ = xy_decel_ = xy_stop_ = start_;
    }

    if (distance_z > 0) {
      // Same for z motion
      plan_trap_velo(cxt_.auv_z_accel_, cxt_.auv_z_velo_, distance_z, plan_.t, z_run_, z_decel_, z_stop_);
      initial_accel_.z = goal_.pose.pose.z > plan_.fp.pose.pose.z ? cxt_.auv_z_accel_ : -cxt_.auv_z_accel_;
    } else {
      z_run_ = z_decel_ = z_stop_ = start_;
    }

    if (distance_yaw > 0) {
      // Same for yaw motion
      plan_trap_velo(cxt_.auv_yaw_accel_, cxt_.auv_yaw_velo_, distance_yaw, plan_.t, yaw_run_, yaw_decel_, yaw_stop_);
      initial_accel_.yaw = norm_angle(goal_.pose.pose.yaw - plan_.fp.pose.pose.yaw) > 0 ?
                           cxt_.auv_yaw_accel_ : -cxt_.auv_yaw_accel_;
    } else {
      yaw_run_ = yaw_decel_ = yaw_stop_ = start_;
    }

    // Start phase 1: accelerate to target velocity
    accel_ = initial_accel_;
  }

  rclcpp::Duration TrapVelo::duration() const
  {
    auto result = xy_stop_ - start_;

    if (z_stop_ - start_ > result) {
      result = z_stop_ - start_;
    }

    if (yaw_stop_ - start_ > result) {
      result = yaw_stop_ - start_;
    }

    return result;
  }

  bool TrapVelo::advance(const rclcpp::Duration &d)
  {
    // Update time
    plan_.t = plan_.t + d;

    // Check for exit condition
    if (plan_.t > xy_stop_ && plan_.t > z_stop_ && plan_.t > yaw_stop_) {
      return false;
    }

    // Update xy motion phase
    if (plan_.t > xy_stop_) {
      // Stop
      accel_.x = 0;
      accel_.y = 0;
      twist_.x = 0;
      twist_.y = 0;
    } else if (plan_.t > xy_decel_) {
      // Start phase 3: decelerate
      accel_.x = -initial_accel_.x;
      accel_.y = -initial_accel_.y;
    } else if (plan_.t > xy_run_) {
      // Start phase 2: run at constant velocity
      accel_.x = 0;
      accel_.y = 0;
      twist_.x = cos(angle_to_goal_) * cxt_.auv_xy_velo_;
      twist_.y = sin(angle_to_goal_) * cxt_.auv_xy_velo_;
    }

    // Same for z motion
    if (plan_.t > z_stop_) {
      // Stop
      accel_.z = 0;
      twist_.z = 0;
    } else if (plan_.t > z_decel_) {
      // Start phase 3: decelerate
      accel_.z = -initial_accel_.z;
    } else if (plan_.t > z_run_) {
      // Start phase 2: run at constant velocity
      accel_.z = 0;
      twist_.z = initial_accel_.z > 0 ? cxt_.auv_z_velo_ : -cxt_.auv_z_velo_;
    }

    // Same for yaw motion
    if (plan_.t > yaw_stop_) {
      // Stop
      accel_.yaw = 0;
      twist_.yaw = 0;
    } else if (plan_.t > yaw_decel_) {
      // Start phase 3: decelerate
      accel_.yaw = -initial_accel_.yaw;
    } else if (plan_.t > yaw_run_) {
      // Start phase 2: run at constant velocity
      accel_.yaw = 0;
      twist_.yaw = initial_accel_.yaw > 0 ? cxt_.auv_yaw_velo_ : -cxt_.auv_yaw_velo_;
    }

#if 1
    // Rotate x and y velocity into the body frame
    double twist_forward, twist_strafe;
    rotate_frame(twist_.x, twist_.y, plan_.fp.pose.pose.yaw, twist_forward, twist_strafe);

    // Compute acceleration due to drag
    auto drag_forward = cxt_.model_.drag_accel_f(twist_forward);
    auto drag_strafe = cxt_.model_.drag_accel_s(twist_strafe);

    // Rotate forward and strafe drag back into the world frame
    rotate_frame(drag_forward, drag_strafe, -plan_.fp.pose.pose.yaw, drag_.x, drag_.y);
#else
    // X and Y drag depend on yaw and direction of motion
    double drag_const_x, drag_const_y;
    cxt_.model_.drag_const_world(plan_.fp.pose.pose.yaw, angle_to_goal_, drag_const_x, drag_const_y);

    // Acceleration due to drag
    drag_.x = cxt_.model_.drag_accel(twist_.x, drag_const_x);
    drag_.y = cxt_.model_.drag_accel(twist_.y, drag_const_y);
#endif
    drag_.z = cxt_.model_.drag_accel(twist_.z, cxt_.model_.drag_const_z());
    drag_.yaw = cxt_.model_.drag_accel_yaw(twist_.yaw);

    // Acceleration due to thrust
    ff_.x = accel_.x - drag_.x;
    ff_.y = accel_.y - drag_.y;
    ff_.z = accel_.z - drag_.z + cxt_.model_.hover_accel_z();
    ff_.yaw = accel_.yaw - drag_.yaw;

    // std::cout << std::fixed << std::setprecision(4) << "ff_: " << ff_ << std::endl;

    auto dt = d.seconds();

    // Twist
    twist_.x += accel_.x * dt;
    twist_.y += accel_.y * dt;
    twist_.z += accel_.z * dt;
    twist_.yaw += accel_.yaw * dt;

    // Plan
    plan_.fp.pose.pose.x += twist_.x * dt;
    plan_.fp.pose.pose.y += twist_.y * dt;
    plan_.fp.pose.pose.z += twist_.z * dt;
    plan_.fp.pose.pose.yaw = norm_angle(plan_.fp.pose.pose.yaw + twist_.yaw * dt);

    return true;
  }

  std::string TrapVelo::to_str()
  {
    auto xy_dt = (xy_stop_ - start_).seconds();
    auto z_dt = (z_stop_ - start_).seconds();
    auto yaw_dt = (yaw_stop_ - start_).seconds();

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2)
       << type_name() << ", xy dt: " << xy_dt
       << ", z dt: " << z_dt
       << ", yaw dt: " << yaw_dt;
    return ss.str();
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_vertical(const AUVContext &cxt, FPStamped &plan, double z)
  {
    FP goal = plan.fp;
    goal.pose.pose.z = z;
    auto result = std::make_shared<TrapVelo>(cxt, orca_msgs::msg::Control::POSE_VERTICAL, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_rotate(const AUVContext &cxt, FPStamped &plan, double yaw)
  {
    FP goal = plan.fp;
    goal.pose.pose.yaw = yaw;
    auto result = std::make_shared<TrapVelo>(cxt, orca_msgs::msg::Control::POSE_ROTATE, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_line(const AUVContext &cxt, FPStamped &plan, double x, double y)
  {
    FP goal = plan.fp;
    goal.pose.pose.x = x;
    goal.pose.pose.y = y;
    auto result = std::make_shared<TrapVelo>(cxt, orca_msgs::msg::Control::POSE_LINE, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_pose(const AUVContext &cxt, FPStamped &plan, const FP &goal)
  {
    auto result = std::make_shared<TrapVelo>(cxt, orca_msgs::msg::Control::POSE_COMBO, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  //=====================================================================================
  // RotateToMarker
  //=====================================================================================

  RotateToMarker::RotateToMarker(const AUVContext &cxt, const ObservationStamped &start, const Observation &goal) :
    ObservationSegmentBase(cxt, orca_msgs::msg::Control::OBS_RTM, start, goal)
  {
    double distance_yaw = std::abs(norm_angle(plan_.o.yaw - goal_.yaw));

    if (distance_yaw > 0) {
      // Plan yaw motion, start phase 1
      plan_trap_velo(cxt_.auv_yaw_accel_, cxt_.auv_yaw_velo_, distance_yaw, plan_.t, yaw_run_, yaw_decel_, yaw_stop_);
      initial_accel_.yaw = norm_angle(goal_.yaw - plan_.o.yaw) > 0 ? cxt_.auv_yaw_accel_ : -cxt_.auv_yaw_accel_;
    } else {
      yaw_run_ = yaw_decel_ = yaw_stop_ = start_;
    }

    // Start phase 1: accelerate to target velocity
    accel_ = initial_accel_;
  }

  std::string RotateToMarker::to_str()
  {
    std::stringstream ss;
    ss << "rotate to marker start: " << plan_ << ", goal: " << goal_;
    return ss.str();
  }

  bool RotateToMarker::advance(const rclcpp::Duration &d)
  {
    // Update time
    plan_.t = plan_.t + d;

    // Check for exit condition
    if (plan_.t > yaw_stop_) {
      return false;
    }

    // Update yaw motion phase
    if (plan_.t > yaw_stop_) {
      // Stop
      accel_.yaw = 0;
      twist_.yaw = 0;
    } else if (plan_.t > yaw_decel_) {
      // Start phase 3: decelerate
      accel_.yaw = -initial_accel_.yaw;
    } else if (plan_.t > yaw_run_) {
      // Start phase 2: run at constant velocity
      accel_.yaw = 0;
    }

    // Acceleration due to drag
    drag_.yaw = cxt_.model_.drag_accel_yaw(twist_.yaw);

    // Acceleration due to thrust
    ff_.yaw = accel_.yaw - drag_.yaw;

    // std::cout << std::fixed << std::setprecision(2) << "ff_: " << ff_ << std::endl;

    auto dt = d.seconds();

    // Twist
    twist_.yaw += accel_.yaw * dt;

    // Plan
    plan_.o.yaw = norm_angle(plan_.o.yaw + twist_.yaw * dt);

    return true;
  }

  //=====================================================================================
  // MoveToMarker
  //=====================================================================================

  MoveToMarker::MoveToMarker(const AUVContext &cxt, const ObservationStamped &start, const Observation &goal) :
    ObservationSegmentBase(cxt, orca_msgs::msg::Control::OBS_MTM, start, goal)
  {
    double distance_fwd = std::abs(plan_.o.distance - goal_.distance);

    if (distance_fwd > 0) {
      // Plan forward motion, start phase 1
      plan_trap_velo(cxt_.auv_xy_accel_, cxt_.auv_xy_velo_, distance_fwd, plan_.t, f_run_, f_decel_, f_stop_);
      initial_accel_.forward = cxt_.auv_xy_accel_; // Always moving foward, so always +accel to start
    } else {
      f_run_ = f_decel_ = f_stop_ = start_;
    }

    // Start phase 1: accelerate to target velocity
    accel_ = initial_accel_;
  }

  std::string MoveToMarker::to_str()
  {
    std::stringstream ss;
    ss << "move to marker start: " << plan_ << ", goal: " << goal_;
    return ss.str();
  }

  bool MoveToMarker::advance(const rclcpp::Duration &d)
  {
    // Update time
    plan_.t = plan_.t + d;

    // Check for exit condition
    if (plan_.t > f_stop_) {
      return false;
    }

    // Update forward motion phase
    if (plan_.t > f_stop_) {
      // Stop
      accel_.forward = 0;
      twist_.forward = 0;
    } else if (plan_.t > f_decel_) {
      // Start phase 3: decelerate
      accel_.forward = -initial_accel_.forward;
    } else if (plan_.t > f_run_) {
      // Start phase 2: run at constant velocity
      accel_.forward = 0;
    }

    // Acceleration due to drag
    drag_.forward = cxt_.model_.drag_accel_f(twist_.forward);

    // Acceleration due to thrust
    ff_.forward = accel_.forward - drag_.forward;

    // std::cout << std::fixed << std::setprecision(2) << "ff_: " << ff_ << std::endl;

    auto dt = d.seconds();

    // Twist
    twist_.forward += accel_.forward * dt;

    // Plan
    plan_.o.distance = plan_.o.distance - twist_.forward * dt;

    return true;
  }

} // namespace orca_base