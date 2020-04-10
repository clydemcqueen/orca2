#include "orca_base/observation_segment.hpp"

#include <iomanip>

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

  //=====================================================================================
  // ObservationSegmentBase
  //=====================================================================================

  ObservationSegmentBase::ObservationSegmentBase(const AUVContext &cxt, uint8_t type, mw::PolarObservationStamped start,
                                                 mw::PolarObservation goal) :
    SegmentBase{cxt, type},
    plan_{std::move(start)},
    goal_{goal}
  {
    // Default ff includes acceleration to counteract buoyancy
    ff_ = mw::AccelerationBody{0, 0, cxt.model_.hover_accel_z(), 0};
  }

  /* Future work... ObservationSegmentBase should look like this (see Trap2):
   *     mw::ObservationStamped o0_;    // Start observation
   *     mw::ObservationStamped o1_;
   *     mw::ObservationStamped o2_;
   *     mw::ObservationStamped o3_;    // End observation
   *     mw::AccelerationBody a0_;      // Acceleration at o0_.t
   *     mw::TwistBody v1_;             // Velocity at o1_.t
   *
   * ... and SegmentBase::advance() should take rclcpp::Time, not rclcpp::Duration
   */

  //=====================================================================================
  // plan_obs_fast
  //=====================================================================================

  void plan_obs_fast(const bool angle, const double accel, const double max_velo, const double distance,
                     const rclcpp::Time &start, rclcpp::Time &run, rclcpp::Time &decel, rclcpp::Time &stop)
  {
    FastPlan plan(angle, distance, accel, max_velo);

    run = start + rclcpp::Duration::from_seconds(plan.t_ramp);
    decel = run + rclcpp::Duration::from_seconds(plan.t_run);
    stop = decel + rclcpp::Duration::from_seconds(plan.t_ramp);
  }

  //=====================================================================================
  // RotateToMarker
  //=====================================================================================

  RotateToMarker::RotateToMarker(const AUVContext &cxt,
                                 const mw::PolarObservationStamped &start,
                                 const mw::PolarObservation &goal) :
    ObservationSegmentBase{cxt, orca_msgs::msg::MissionState::OBS_RTM, start, goal}
  {
    double distance_yaw = std::abs(orca::norm_angle(plan_.observation().bearing() - goal_.bearing()));

    if (distance_yaw > 0) {
      // Plan yaw motion, start phase 1
      plan_obs_fast(true, cxt_.mtm_yaw_accel_, cxt_.mtm_yaw_velo_, distance_yaw, plan_.header().t(),
                    yaw_run_, yaw_decel_, yaw_stop_);
      initial_accel_.yaw() = orca::norm_angle(goal_.bearing() - plan_.observation().bearing()) > 0 ?
                             cxt_.mtm_yaw_accel_ : -cxt_.mtm_yaw_accel_;
    } else {
      yaw_run_ = yaw_decel_ = yaw_stop_ = start_;
    }

    // Start phase 1: accelerate to target velocity
    accel_ = initial_accel_;
  }

  std::string RotateToMarker::to_str()
  {
    std::stringstream ss;
    ss << type_name() << ", start: " << plan_ << ", goal: " << goal_;
    return ss.str();
  }

  bool RotateToMarker::advance(const rclcpp::Duration &d)
  {
    // Update time
    plan_.header().t() = plan_.header().t() + d;

    // Check for exit condition
    if (plan_.header().t() > yaw_stop_) {
      return false;
    }

    // Update yaw motion phase
    if (plan_.header().t() > yaw_stop_) {
      // Stop
      accel_.yaw() = 0;
      twist_.yaw() = 0;
    } else if (plan_.header().t() > yaw_decel_) {
      // Start phase 3: decelerate
      accel_.yaw() = -initial_accel_.yaw();
    } else if (plan_.header().t() > yaw_run_) {
      // Start phase 2: run at constant velocity
      accel_.yaw() = 0;
    }

    // Acceleration due to drag
    drag_.yaw() = cxt_.model_.drag_accel_yaw(twist_.yaw());

    // Acceleration due to thrust
    ff_.yaw() = accel_.yaw() - drag_.yaw();

    // std::cout << std::fixed << std::setprecision(2) << "ff_: " << ff_ << std::endl;

    auto dt = d.seconds();

    // Twist
    twist_.yaw() += accel_.yaw() * dt;

    // Plan
    plan_.observation().bearing() = orca::norm_angle(plan_.observation().bearing() + twist_.yaw() * dt);

    return true;
  }

  //=====================================================================================
  // MoveToMarker
  //=====================================================================================

  MoveToMarker::MoveToMarker(const AUVContext &cxt, const mw::PolarObservationStamped &start,
                             const mw::PolarObservation &goal) :
    ObservationSegmentBase(cxt, orca_msgs::msg::MissionState::OBS_MTM, start, goal)
  {
    double distance_fwd = std::abs(plan_.observation().distance() - goal_.distance());

    if (distance_fwd > 0) {
      // Plan forward motion, start phase 1
      plan_obs_fast(false, cxt_.mtm_fwd_accel_, cxt_.mtm_fwd_velo_, distance_fwd, plan_.header().t(),
                    f_run_, f_decel_, f_stop_);
      initial_accel_.forward() = cxt_.mtm_fwd_accel_; // Always moving foward, so always +accel to start
    } else {
      f_run_ = f_decel_ = f_stop_ = start_;
    }

    // Start phase 1: accelerate to target velocity
    accel_ = initial_accel_;
  }

  std::string MoveToMarker::to_str()
  {
    std::stringstream ss;
    ss << type_name() << ", start: " << plan_ << ", goal: " << goal_;
    return ss.str();
  }

  bool MoveToMarker::advance(const rclcpp::Duration &d)
  {
    // Update time
    plan_.header().t() = plan_.header().t() + d;

    // Check for exit condition
    if (plan_.header().t() > f_stop_) {
      return false;
    }

    // Update forward motion phase
    if (plan_.header().t() > f_stop_) {
      // Stop
      accel_.forward() = 0;
      twist_.forward() = 0;
    } else if (plan_.header().t() > f_decel_) {
      // Start phase 3: decelerate
      accel_.forward() = -initial_accel_.forward();
    } else if (plan_.header().t() > f_run_) {
      // Start phase 2: run at constant velocity
      accel_.forward() = 0;
    }

    // Acceleration due to drag
    drag_.forward() = cxt_.model_.drag_accel_f(twist_.forward());

    // Acceleration due to thrust
    ff_.forward() = accel_.forward() - drag_.forward();

    // std::cout << std::fixed << std::setprecision(2) << "ff_: " << ff_ << std::endl;

    auto dt = d.seconds();

    // Twist
    twist_.forward() += accel_.forward() * dt;

    // Plan
    plan_.observation().distance() = plan_.observation().distance() - twist_.forward() * dt;

    return true;
  }

} // namespace orca_base