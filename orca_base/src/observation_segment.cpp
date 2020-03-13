#include "orca_base/observation_segment.hpp"

#include <iomanip>

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

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
    ff_ = orca::AccelerationBody{0, 0, cxt.model_.hover_accel_z(), 0};
  }

  //=====================================================================================
  // plan_trap_velo TODO use FastPlan instead
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

  //=====================================================================================
  // RotateToMarker
  //=====================================================================================

  RotateToMarker::RotateToMarker(const AUVContext &cxt, const ObservationStamped &start, const Observation &goal) :
    ObservationSegmentBase(cxt, orca_msgs::msg::Control::OBS_RTM, start, goal)
  {
    double distance_yaw = std::abs(orca::norm_angle(plan_.o.yaw - goal_.yaw));

    if (distance_yaw > 0) {
      // Plan yaw motion, start phase 1
      plan_trap_velo(cxt_.mtm_yaw_accel_, cxt_.mtm_yaw_velo_, distance_yaw, plan_.t, yaw_run_, yaw_decel_, yaw_stop_);
      initial_accel_.yaw = orca::norm_angle(goal_.yaw - plan_.o.yaw) > 0 ? cxt_.mtm_yaw_accel_ : -cxt_.mtm_yaw_accel_;
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
    plan_.o.yaw = orca::norm_angle(plan_.o.yaw + twist_.yaw * dt);

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
      plan_trap_velo(cxt_.mtm_fwd_accel_, cxt_.mtm_fwd_velo_, distance_fwd, plan_.t, f_run_, f_decel_, f_stop_);
      initial_accel_.forward = cxt_.mtm_fwd_accel_; // Always moving foward, so always +accel to start
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