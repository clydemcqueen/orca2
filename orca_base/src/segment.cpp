#include "orca_base/segment.hpp"

#include <iomanip>

#include "orca_shared/util.hpp"

using namespace orca;

namespace orca_base
{

  //=====================================================================================
  // SegmentBase
  //=====================================================================================

  SegmentBase::SegmentBase(const rclcpp::Logger &logger, BaseContext cxt) :
    logger_{logger},
    cxt_{std::move(cxt)}
  {
  }

  //=====================================================================================
  // PoseSegmentBase
  //=====================================================================================

  PoseSegmentBase::PoseSegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped start, FP goal) :
    SegmentBase{logger, cxt},
    plan_{std::move(start)},
    goal_{std::move(goal)}
  {
    // Default ff includes acceleration to counteract buoyancy
    ff_ = Acceleration{0, 0, cxt.model_.hover_accel_z(), 0};
  }

  //=====================================================================================
  // ObservationSegmentBase
  //=====================================================================================

  ObservationSegmentBase::ObservationSegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt,
                                                 ObservationStamped start, Observation goal) :
    SegmentBase{logger, cxt},
    plan_{std::move(start)},
    goal_{std::move(goal)}
  {
    // Default ff includes acceleration to counteract buoyancy
    ff_ = AccelerationBody{0, 0, cxt.model_.hover_accel_z(), 0};
  }

  //=====================================================================================
  // Pause
  //=====================================================================================

  Pause::Pause(const rclcpp::Logger &logger, const BaseContext &cxt, const FPStamped &start,
               const rclcpp::Duration &d) :
    PoseSegmentBase{logger, cxt, start, start.fp}, d_{d}
  {}

  void Pause::log_info()
  {
    RCLCPP_INFO(logger_, "pause for %g seconds", d_.seconds());
  }

  bool Pause::advance(const rclcpp::Duration &d)
  {
    // Update plan
    plan_.t = plan_.t + d;

    // Count down time remaining
    d_ = d_ - d;

    return d_.nanoseconds() > 0;
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

  void plan_trap_velo(double accel, double max_velo, double distance,
                      const rclcpp::Time &start, rclcpp::Time &run, rclcpp::Time &decel, rclcpp::Time &stop)
  {
    auto ramp_seconds = max_velo / accel;
    auto ramp_distance = accel * ramp_seconds * ramp_seconds / 2;

    double run_seconds;
    if (2 * ramp_distance < distance) {
      run_seconds = (distance - 2 * ramp_distance) / max_velo;
    } else {
      // Distance too short, will not hit max_velo
      ramp_seconds = sqrt(distance / accel);
      run_seconds = 0;
    }

    run = start + rclcpp::Duration::from_seconds(ramp_seconds);
    decel = run + rclcpp::Duration::from_seconds(run_seconds);
    stop = decel + rclcpp::Duration::from_seconds(ramp_seconds);
  }

  TrapVelo::TrapVelo(const rclcpp::Logger &logger, const BaseContext &cxt, const FPStamped &start, const FP &goal) :
    PoseSegmentBase{logger, cxt, start, goal}, angle_to_goal_{0}, start_{start.t}
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
    }

    // X and Y drag depend on yaw and direction of motion
    double linear_drag_x, linear_drag_y;
    cxt_.model_.linear_drag_world(plan_.fp.pose.pose.yaw, angle_to_goal_, linear_drag_x, linear_drag_y);

    // Acceleration due to drag
    drag_.x = cxt_.model_.drag_accel(twist_.x, linear_drag_x);
    drag_.y = cxt_.model_.drag_accel(twist_.y, linear_drag_y);
    drag_.z = cxt_.model_.drag_accel(twist_.z, cxt_.model_.linear_drag_z());
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

  void TrapVelo::log_info()
  {
    auto xy_seconds = (xy_stop_ - start_).seconds();
    auto z_seconds = (z_stop_ - start_).seconds();
    auto yaw_seconds = (yaw_stop_ - start_).seconds();

    RCLCPP_INFO_STREAM(logger_, std::fixed << std::setprecision(4)
                                           << "cv xy_seconds: " << xy_seconds
                                           << ", z_seconds: " << z_seconds
                                           << ", yaw_seconds: " << yaw_seconds
                                           << ", initial_accel_: " << initial_accel_);
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_vertical(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped &plan, double z)
  {
    FP goal = plan.fp;
    goal.pose.pose.z = z;
    auto result = std::make_shared<TrapVelo>(logger, cxt, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_rotate(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped &plan, double yaw)
  {
    FP goal = plan.fp;
    goal.pose.pose.yaw = yaw;
    auto result = std::make_shared<TrapVelo>(logger, cxt, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_line(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped &plan, double x, double y)
  {
    FP goal = plan.fp;
    goal.pose.pose.x = x;
    goal.pose.pose.y = y;
    auto result = std::make_shared<TrapVelo>(logger, cxt, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_pose(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped &plan, const FP &goal)
  {
    auto result = std::make_shared<TrapVelo>(logger, cxt, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  //=====================================================================================
  // RotateToMarker
  //=====================================================================================

  RotateToMarker::RotateToMarker(const rclcpp::Logger &logger, const BaseContext &cxt,
                                 const ObservationStamped &start, const Observation &goal) :
    ObservationSegmentBase(logger, cxt, start, goal)
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

  void RotateToMarker::log_info()
  {
    RCLCPP_INFO_STREAM(logger_, "rotate to marker start: " << plan_ << ", goal: " << goal_);
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
    // ff_.vertical = accel_.vertical - drag_.vertical + cxt_.model_.hover_accel_z();
    ff_.vertical = cxt_.model_.hover_accel_z();
    ff_.yaw = accel_.yaw - drag_.yaw;

    // std::cout << std::fixed << std::setprecision(4) << "ff_: " << ff_ << std::endl;

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

  MoveToMarker::MoveToMarker(const rclcpp::Logger &logger, const BaseContext &cxt,
                             const ObservationStamped &start, const Observation &goal) :
    ObservationSegmentBase(logger, cxt, start, goal)
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

  void MoveToMarker::log_info()
  {
    RCLCPP_INFO_STREAM(logger_, "move to marker start: " << plan_ << ", goal: " << goal_);
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
    // ff_.vertical = accel_.vertical - drag_.vertical + cxt_.model_.hover_accel_z();
    ff_.vertical = cxt_.model_.hover_accel_z();
    ff_.forward = accel_.forward - drag_.forward;

    // std::cout << std::fixed << std::setprecision(4) << "ff_: " << ff_ << std::endl;

    auto dt = d.seconds();

    // Twist
    twist_.forward += accel_.forward * dt;

    // Plan
    plan_.o.distance = plan_.o.distance - twist_.forward * dt;

    return true;
  }

} // namespace orca_base