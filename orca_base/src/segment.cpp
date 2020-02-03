#include "orca_base/segment.hpp"

#include <utility>

using namespace orca;

namespace orca_base
{

  // Run some numerical approximations
  constexpr int NUM_MAX_TICKS = 100;    // Max number of ticks
  constexpr double NUM_DT = 0.1;        // Time step for each tick
  constexpr double END_VELO_XY = 0.1;   // When xy velo is < this, end the simulation
  constexpr double END_VELO_Z = 0.1;    // When z velo is < this, end the simulation
  constexpr double END_VELO_YAW = 0.1;  // When yaw velo is < this, end the simulation

  // Move this to orca::FP
  bool find_obs(const FP &fp, int marker_id, Observation &obs)
  {
    for (const auto &i : fp.observations) {
      if (i.id == marker_id) {
        obs = i;
        return true;
      }
    }

    return false;
  }

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
                                   orca::FP start, orca::FP goal) :
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

  Pause::Pause(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start, double seconds) :
    PoseSegmentBase(logger, cxt, start, start),
    seconds_{seconds}
  {}

  void Pause::log_info()
  {
    RCLCPP_INFO(logger_, "pause for %g seconds", seconds_);
  }

  bool Pause::advance(double dt)
  {
    seconds_ -= dt;

    return seconds_ > 0;
  }

  //=====================================================================================
  // VerticalSegment
  //=====================================================================================

  // Compute the deceleration (glide) distance
  double deceleration_distance_z(const BaseContext &cxt, double velo_z)
  {
    double z = 0;

    for (int ticks = 0; ticks < NUM_MAX_TICKS; ++ticks) {
      // Compute drag force
      double accel_drag_z = Model::force_to_accel(-cxt.model_.drag_force_z(velo_z));

      // Update velocity
      velo_z -= accel_drag_z * NUM_DT;

      // Update distance
      z += velo_z * NUM_DT;

      if (std::abs(velo_z) < END_VELO_Z) {
        // Close enough
        return std::abs(z);
      }
    }

    std::cout << "deceleration_distance_z failed" << std::endl;
    return 0;
  }

  VerticalSegment::VerticalSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start,
                                   const orca::FP &goal) :
    PoseSegmentBase(logger, cxt, start, goal)
  {
    assert(start.pose.pose.distance_xy(goal.pose.pose) < EPSILON_PLAN_XYZ &&
           start.pose.pose.distance_yaw(goal.pose.pose) < EPSILON_PLAN_YAW);

    // Target velocity
    double velo_z = goal.pose.pose.z > start.pose.pose.z ? cxt.auv_z_speed_ : -cxt.auv_z_speed_;

    // Drag force => thrust force => acceleration => feedforward
    // Add to the hover acceleration
    ff_.z += Model::force_to_accel(-cxt.model_.drag_force_z(velo_z));
  }

  void VerticalSegment::log_info()
  {
    RCLCPP_INFO(logger_, "vertical: start %g, goal %g, ff %g", plan_.pose.pose.z, goal_.pose.pose.z, ff_.z);
  }

  bool VerticalSegment::advance(double dt)
  {
    double distance_remaining = plan_.pose.pose.distance_z(goal_.pose.pose);
    if (distance_remaining > EPSILON_PLAN_XYZ) {
      if (distance_remaining - deceleration_distance_z(cxt_, twist_.z) < EPSILON_PLAN_XYZ) {
        // Decelerate
        ff_.z = cxt_.model_.hover_accel_z();
      }

      // Compute acceleration due to drag
      double accel_drag_z = Model::force_to_accel(-cxt_.model_.drag_force_z(twist_.z));

      // Update velocity
      twist_.z += (ff_.z - cxt_.model_.hover_accel_z() - accel_drag_z) * dt;

      // Update plan
      plan_.pose.pose.z += twist_.z * dt;

      return true;
    } else {
      return false;
    }
  }

  //=====================================================================================
  // RotateSegment
  //=====================================================================================

  // Compute the deceleration (glide) distance
  double deceleration_distance_yaw(const BaseContext &cxt, double velo_yaw)
  {
    double yaw = 0;

    for (int ticks = 0; ticks < NUM_MAX_TICKS; ++ticks) {
      // Compute drag force
      double accel_drag_yaw = Model::torque_to_accel_yaw(-cxt.model_.drag_torque_yaw(velo_yaw));

      // Update velocity
      velo_yaw -= accel_drag_yaw * NUM_DT;

      // Update distance
      yaw = norm_angle(yaw + velo_yaw * NUM_DT);

      if (std::abs(velo_yaw) < END_VELO_YAW) {
        // Close enough
        return std::abs(yaw);
      }
    }

    std::cout << "deceleration_distance_yaw failed" << std::endl;
    return 0;
  }

  RotateSegment::RotateSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start,
                               const orca::FP &goal) :
    PoseSegmentBase(logger, cxt, start, goal)
  {
    assert(start.pose.pose.distance_xy(goal.pose.pose) < EPSILON_PLAN_XYZ ||
           start.pose.pose.distance_z(goal.pose.pose) < EPSILON_PLAN_XYZ);

    // Target velocity
    double velo_yaw =
      norm_angle(goal.pose.pose.yaw - start.pose.pose.yaw) > 0 ? cxt.auv_yaw_speed_ : -cxt.auv_yaw_speed_;

    // Drag torque => thrust torque => acceleration => feedforward
    ff_.yaw = Model::torque_to_accel_yaw(-cxt.model_.drag_torque_yaw(velo_yaw));
  }

  void RotateSegment::log_info()
  {
    RCLCPP_INFO(logger_, "rotate: start %g, goal %g, ff %g", plan_.pose.pose.yaw, goal_.pose.pose.yaw, ff_.yaw);
  }

  bool RotateSegment::advance(double dt)
  {
    double distance_remaining = plan_.pose.pose.distance_yaw(goal_.pose.pose);
    if (distance_remaining > EPSILON_PLAN_YAW) {
      if (distance_remaining - deceleration_distance_yaw(cxt_, twist_.yaw) < EPSILON_PLAN_YAW) {
        // Decelerate
        ff_.yaw = 0;
      }

      // Compute acceleration due to drag
      double accel_drag_yaw = Model::torque_to_accel_yaw(-cxt_.model_.drag_torque_yaw(twist_.yaw));

      // Update velocity
      twist_.yaw += (ff_.yaw - accel_drag_yaw) * dt;

      // Update plan
      plan_.pose.pose.yaw = norm_angle(plan_.pose.pose.yaw + twist_.yaw * dt);

      return true;
    } else {
      return false;
    }
  }

  //=====================================================================================
  // LineSegment
  //=====================================================================================

  // Compute acceleration required to counteract drag force
  void drag_force_to_accel_xy(const BaseContext &cxt, const double yaw, const double x_v, const double y_v,
                              double &x_a, double &y_a)
  {
    // Rotate velocity into the body frame
    double forward_v, strafe_v;
    rotate_frame(x_v, y_v, yaw, forward_v, strafe_v);

    // Calc acceleration due to drag force
    double forward_a = Model::force_to_accel(-cxt.model_.drag_force_x(forward_v));
    double strafe_a = Model::force_to_accel(-cxt.model_.drag_force_y(strafe_v));

    // Rotate back
    rotate_frame(forward_a, strafe_a, -yaw, x_a, y_a);
  }

  // Compute the deceleration (glide) distance
  double deceleration_distance_xy(const BaseContext &cxt, const double yaw, double velo_x, double velo_y)
  {
    double x = 0;
    double y = 0;

    for (int ticks = 0; ticks < NUM_MAX_TICKS; ++ticks) {
      // Compute drag force
      double accel_drag_x, accel_drag_y;
      drag_force_to_accel_xy(cxt, yaw, velo_x, velo_y, accel_drag_x, accel_drag_y);

      // Update velocity
      velo_x -= accel_drag_x * NUM_DT;
      velo_y -= accel_drag_y * NUM_DT;

      // Update distance
      x += velo_x * NUM_DT;
      y += velo_y * NUM_DT;

      if (std::hypot(velo_x, velo_y) < END_VELO_XY) {
        // Close enough
        return std::hypot(x, y);
      }
    }

    std::cout << "deceleration_distance_xy failed" << std::endl;
    return 0;
  }

  LineSegment::LineSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FP &start,
                           const orca::FP &goal) :
    PoseSegmentBase(logger, cxt, start, goal)
  {
    assert(start.pose.pose.distance_z(goal.pose.pose) < EPSILON_PLAN_XYZ &&
           start.pose.pose.distance_yaw(goal.pose.pose) < EPSILON_PLAN_YAW);

    // Compute angle plan_ => goal_
    double angle_to_goal = atan2(goal_.pose.pose.y - plan_.pose.pose.y, goal_.pose.pose.x - plan_.pose.pose.x);

    // Drag force => thrust force => acceleration => feedforward
    drag_force_to_accel_xy(cxt_, goal_.pose.pose.yaw, cxt_.auv_xy_speed_ * cos(angle_to_goal),
                           cxt_.auv_xy_speed_ * sin(angle_to_goal), ff_.x, ff_.y);
  }

  void LineSegment::log_info()
  {
    RCLCPP_INFO(logger_, "line: start (%g, %g), goal (%g, %g), ff (%g, %g)",
                plan_.pose.pose.x, plan_.pose.pose.y, goal_.pose.pose.x, goal_.pose.pose.y, ff_.x, ff_.y);
  }

  bool LineSegment::advance(double dt)
  {
    double remaining = plan_.pose.pose.distance_xy(goal_.pose.pose);
    if (remaining > EPSILON_PLAN_XYZ) {
      if (remaining - deceleration_distance_xy(cxt_, goal_.pose.pose.yaw, twist_.x, twist_.y) < EPSILON_PLAN_XYZ) {
        // Decelerate
        ff_.x = ff_.y = 0;
      }

      // Compute acceleration due to drag
      double accel_drag_x, accel_drag_y;
      drag_force_to_accel_xy(cxt_, goal_.pose.pose.yaw, twist_.x, twist_.y, accel_drag_x, accel_drag_y);

      // Update velocity
      twist_.x += (ff_.x - accel_drag_x) * dt;
      twist_.y += (ff_.y - accel_drag_y) * dt;

      // Update plan
      plan_.pose.pose.x += twist_.x * dt;
      plan_.pose.pose.y += twist_.y * dt;

      return true;
    } else {
      return false;
    }
  }

  //=====================================================================================
  // MoveToMarkerSegment
  //=====================================================================================

  // Compute the deceleration (glide) distance
  double deceleration_distance_forward(const BaseContext &cxt, double velo_x)
  {
    double x = 0;

    for (int ticks = 0; ticks < NUM_MAX_TICKS; ++ticks) {
      // Compute drag force
      double accel_drag_x = Model::force_to_accel(-cxt.model_.drag_force_x(velo_x));

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

  MoveToMarkerSegment::MoveToMarkerSegment(const rclcpp::Logger &logger, const BaseContext &cxt,
                                           const orca::Observation &start, const orca::Observation &goal) :
    ObservationSegmentBase(logger, cxt, start, goal)
  {
    // Forward acceleration
    ff_.forward = Model::force_to_accel(-cxt_.model_.drag_force_x(cxt_.auv_xy_speed_));

    // Counteract buoyancy
    ff_.vertical = cxt_.model_.hover_accel_z();
  }

  void MoveToMarkerSegment::log_info()
  {
    RCLCPP_INFO_STREAM(logger_, "move to marker start: " << plan_ << ", goal: " << goal_);
  }

  bool MoveToMarkerSegment::advance(double dt)
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
      double accel_drag_x = Model::force_to_accel(-cxt_.model_.drag_force_x(twist_.forward));

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
  // RotateToMarkerSegment
  //=====================================================================================

  RotateToMarkerSegment::RotateToMarkerSegment(const rclcpp::Logger &logger, const BaseContext &cxt,
                                               const orca::Observation &start, const orca::Observation &goal) :
    ObservationSegmentBase(logger, cxt, start, goal)
  {
    // Target velocity
    double velo_yaw = norm_angle(goal.yaw - start.yaw) > 0 ? cxt.auv_yaw_speed_ : -cxt.auv_yaw_speed_;

    // Drag torque => thrust torque => acceleration => feedforward
    ff_.yaw = Model::torque_to_accel_yaw(-cxt.model_.drag_torque_yaw(velo_yaw));

    // Counteract buoyancy
    ff_.vertical = cxt_.model_.hover_accel_z();
  }

  void RotateToMarkerSegment::log_info()
  {
    RCLCPP_INFO_STREAM(logger_, "rotate to marker start: " << plan_ << ", goal: " << goal_);
  }

  bool RotateToMarkerSegment::advance(double dt)
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