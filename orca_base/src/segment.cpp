#include "orca_base/segment.hpp"

#include <iomanip>

using namespace orca;

namespace orca_base
{

  // Run some numerical approximations
  constexpr int NUM_MAX_TICKS = 100;    // Max number of ticks
  constexpr double NUM_DT = 0.1;        // Time step for each tick
  constexpr double END_VELO_XY = 0.1;   // When xy velo is < this, end the simulation
  constexpr double END_VELO_Z = 0.1;    // When z velo is < this, end the simulation
  constexpr double END_VELO_YAW = 0.1;  // When yaw velo is < this, end the simulation

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

  PoseSegmentBase::PoseSegmentBase(const rclcpp::Logger &logger, const BaseContext &cxt,
                                   orca::FPStamped start, orca::FP goal) :
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
                                                 orca::Observation start, orca::Observation goal) :
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

  Pause::Pause(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FPStamped &start,
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

  TrapVelo::TrapVelo(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FPStamped &start,
                     const orca::FP &goal) :
    PoseSegmentBase{logger, cxt, start, goal}, angle_to_goal_{0}
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
      xy_run_ = xy_decel_ = xy_stop_ = plan_.t;
    }

    if (distance_z > 0) {
      // Same for z motion
      plan_trap_velo(cxt_.auv_z_accel_, cxt_.auv_z_velo_, distance_z, plan_.t, z_run_, z_decel_, z_stop_);
      initial_accel_.z = goal_.pose.pose.z > plan_.fp.pose.pose.z ? cxt_.auv_z_accel_ : -cxt_.auv_z_accel_;
    } else {
      z_run_ = z_decel_ = z_stop_ = plan_.t;
    }

    if (distance_yaw > 0) {
      // Same for yaw motion
      plan_trap_velo(cxt_.auv_yaw_accel_, cxt_.auv_yaw_velo_, distance_yaw, plan_.t, yaw_run_, yaw_decel_, yaw_stop_);
      initial_accel_.yaw = norm_angle(goal_.pose.pose.yaw - plan_.fp.pose.pose.yaw) > 0 ?
                           cxt_.auv_yaw_accel_ : -cxt_.auv_yaw_accel_;
    } else {
      yaw_run_ = yaw_decel_ = yaw_stop_ = plan_.t;
    }

    // Start phase 1: accelerate to target velocity
    accel_ = initial_accel_;
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

    // Pose
    plan_.fp.pose.pose.x += twist_.x * dt;
    plan_.fp.pose.pose.y += twist_.y * dt;
    plan_.fp.pose.pose.z += twist_.z * dt;
    plan_.fp.pose.pose.yaw = norm_angle(plan_.fp.pose.pose.yaw + twist_.yaw * dt);

    return true;
  }

  void TrapVelo::log_info()
  {
    auto xy_seconds = (xy_stop_ - plan_.t).seconds();
    auto z_seconds = (z_stop_ - plan_.t).seconds();
    auto yaw_seconds = (yaw_stop_ - plan_.t).seconds();

    RCLCPP_INFO_STREAM(logger_, std::fixed << std::setprecision(4)
      << "cv xy_seconds: " << xy_seconds
      << ", z_seconds: " << z_seconds
      << ", yaw_seconds: " << yaw_seconds
      << ", initial_accel_: " << initial_accel_);
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_vertical(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FPStamped &plan, double z)
  {
    FP goal = plan.fp;
    goal.pose.pose.z = z;
    auto result = std::make_shared<TrapVelo>(logger, cxt, plan, goal);
    plan.fp = goal;
    return result;
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_rotate(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FPStamped &plan, double yaw)
  {
    FP goal = plan.fp;
    goal.pose.pose.yaw = yaw;
    auto result = std::make_shared<TrapVelo>(logger, cxt, plan, goal);
    plan.fp = goal;
    return result;
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_line(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FPStamped &plan, double x,
                      double y)
  {
    FP goal = plan.fp;
    goal.pose.pose.x = x;
    goal.pose.pose.y = y;
    auto result = std::make_shared<TrapVelo>(logger, cxt, plan, goal);
    plan.fp = goal;
    return result;
  }

  std::shared_ptr<TrapVelo>
  TrapVelo::make_pose(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped &plan,
                      const orca::FP &goal)
  {
    auto result = std::make_shared<TrapVelo>(logger, cxt, plan, goal);
    plan.fp = goal;
    return result;
  }

  //=====================================================================================
  // MoveToMarker
  //=====================================================================================

  // Compute the deceleration (glide) distance
  double deceleration_distance_forward(const BaseContext &cxt, double velo_x)
  {
    double x = 0;

    for (int ticks = 0; ticks < NUM_MAX_TICKS; ++ticks) {
      // Compute drag force
      double accel_drag_x = Model::force_to_accel(-cxt.model_.drag_force_f(velo_x));  // TODO f or x???

      // Update velocity
      velo_x -= accel_drag_x * NUM_DT;

      // Update distance
      x += velo_x * NUM_DT;

      if (velo_x < END_VELO_XY) {
        // Close enough
        return x;
      }
    }

    std::cout << "deceleration_distance_forward failed" << std::endl;
    return 0;
  }

  MoveToMarker::MoveToMarker(const rclcpp::Logger &logger, const BaseContext &cxt,
                             const orca::Observation &start, const orca::Observation &goal) :
    ObservationSegmentBase(logger, cxt, start, goal)
  {
    // Forward acceleration
    ff_.forward = Model::force_to_accel(-cxt_.model_.drag_force_f(cxt_.auv_xy_velo_));

    // Counteract buoyancy
    ff_.vertical = cxt_.model_.hover_accel_z();
  }

  void MoveToMarker::log_info()
  {
    RCLCPP_INFO_STREAM(logger_, "move to marker start: " << plan_ << ", goal: " << goal_);
  }

  bool MoveToMarker::advance(const rclcpp::Duration &d)
  {
    // Moving forward is +x but -distance
    double distance_remaining = plan_.distance - goal_.distance;

    if (distance_remaining > EPSILON_PLAN_XYZ) {
#if 0
      if (distance_remaining - deceleration_distance_forward(cxt_, twist_.forward) < EPSILON_PLAN_XYZ) {
        // Decelerate
        ff_.forward = 0;
      }
#endif

      // Compute acceleration due to drag
      double accel_drag_x = Model::force_to_accel(-cxt_.model_.drag_force_f(twist_.forward));  // TODO f or x???

      auto dt = d.seconds();

      // Update velocity
      twist_.forward += (ff_.forward - accel_drag_x) * dt;

      // Update plan
      plan_.distance -= twist_.forward * dt;

      return true;
    } else {
      plan_ = goal_;
      plan_ = goal_;
      twist_ = TwistBody{};
      return false;
    }
  }

  //=====================================================================================
  // RotateToMarker
  //=====================================================================================

  RotateToMarker::RotateToMarker(const rclcpp::Logger &logger, const BaseContext &cxt,
                                 const orca::Observation &start, const orca::Observation &goal) :
    ObservationSegmentBase(logger, cxt, start, goal)
  {
    // Target velocity
    double velo_yaw = norm_angle(goal.yaw - start.yaw) > 0 ? cxt.auv_yaw_velo_ : -cxt.auv_yaw_velo_;

    // Drag torque => thrust torque => acceleration => feedforward
    ff_.yaw = Model::torque_to_accel_yaw(-cxt.model_.drag_torque_yaw(velo_yaw));

    // Counteract buoyancy
    ff_.vertical = cxt_.model_.hover_accel_z();
  }

  void RotateToMarker::log_info()
  {
    RCLCPP_INFO_STREAM(logger_, "rotate to marker start: " << plan_ << ", goal: " << goal_);
  }

  bool RotateToMarker::advance(const rclcpp::Duration &d)
  {
    double distance_remaining = std::abs(norm_angle(goal_.yaw - plan_.yaw));
    if (distance_remaining > EPSILON_PLAN_YAW) {
#if 0
      if (distance_remaining - deceleration_distance_yaw(cxt_, twist_.yaw) < EPSILON_PLAN_YAW) {
        // Decelerate
        ff_.yaw = 0;
      }
#endif

      // Compute acceleration due to drag
      double accel_drag_yaw = Model::torque_to_accel_yaw(-cxt_.model_.drag_torque_yaw(twist_.yaw));

      auto dt = d.seconds();

      // Update velocity
      twist_.yaw += (ff_.yaw - accel_drag_yaw) * dt;

      // Update plan
      plan_.yaw = norm_angle(plan_.yaw + twist_.yaw * dt);

      return true;
    } else {
      return false;
    }
  }

} // namespace orca_base