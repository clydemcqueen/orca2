#include "orca_base/segment.hpp"

namespace orca_base
{

  // Run some numerical approximations
  constexpr int NUM_MAX_TICKS = 100;    // Max number of ticks
  constexpr double NUM_DT = 0.1;        // Time step for each tick
  constexpr double END_VELO_XY = 0.1;   // When xy velo is < this, end the simulation
  constexpr double END_VELO_Z = 0.1;    // When z velo is < this, end the simulation
  constexpr double END_VELO_YAW = 0.1;  // When yaw velo is < this, end the simulation

  //=====================================================================================
  // BaseSegment
  //=====================================================================================

  BaseSegment::BaseSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal) :
    logger_{logger},
    cxt_{cxt},
    plan_{start},
    goal_{goal}
  {
    twist_ = Twist{};

    // Default ff includes acceleration to counteract buoyancy
    ff_ = Acceleration{0, 0, cxt.model_.hover_accel_z(), 0};
  }

  bool BaseSegment::advance(double dt)
  {
    (void) dt;

    return true;
  }

  void BaseSegment::finish()
  {
    plan_ = goal_;
    twist_ = Twist{};
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

  VerticalSegment::VerticalSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start,
                                   const Pose &goal) :
    BaseSegment(logger, cxt, start, goal)
  {
    assert(start.distance_xy(goal) < EPSILON_PLAN_XYZ && start.distance_yaw(goal) < EPSILON_PLAN_YAW);

    // Target velocity
    double velo_z = goal.z > start.z ? cxt.auv_z_speed_ : -cxt.auv_z_speed_;

    // Drag force => thrust force => acceleration => feedforward
    // Add to the hover acceleration
    ff_.z += Model::force_to_accel(-cxt.model_.drag_force_z(velo_z));

    RCLCPP_INFO(logger_, "vertical: start %g, goal %g, ff %g", plan_.z, goal_.z, ff_.z);
  }

  bool VerticalSegment::advance(double dt)
  {
    double distance_remaining = plan_.distance_z(goal_);
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
      plan_.z += twist_.z * dt;

      return true;
    } else {
      finish();
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

  RotateSegment::RotateSegment(const rclcpp::Logger &logger, const BaseContext &cxt,
                               const Pose &start, const Pose &goal) : BaseSegment(logger, cxt, start, goal)
  {
    assert(start.distance_xy(goal) < EPSILON_PLAN_XYZ || start.distance_z(goal) < EPSILON_PLAN_XYZ);

    // Target velocity
    double velo_yaw = norm_angle(goal.yaw - start.yaw) > 0 ? cxt.auv_yaw_speed_ : -cxt.auv_yaw_speed_;

    // Drag torque => thrust torque => acceleration => feedforward
    ff_.yaw = Model::torque_to_accel_yaw(-cxt.model_.drag_torque_yaw(velo_yaw));

    RCLCPP_INFO(logger_, "rotate: start %g, goal %g, ff %g", plan_.yaw, goal_.yaw, ff_.yaw);
  }

  bool RotateSegment::advance(double dt)
  {
    double distance_remaining = plan_.distance_yaw(goal_);
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
      plan_.yaw = norm_angle(plan_.yaw + twist_.yaw * dt);

      return true;
    } else {
      finish();
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

  LineSegment::LineSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const Pose &start, const Pose &goal) :
    BaseSegment(logger, cxt, start, goal)
  {
    assert(start.distance_z(goal) < EPSILON_PLAN_XYZ && start.distance_yaw(goal) < EPSILON_PLAN_YAW);
    init();
  }

  bool LineSegment::extend(const Pose &start, const Pose &goal)
  {
    // Simple test: is this a line segment?
    if (start.distance_yaw(goal) < EPSILON_PLAN_YAW &&
        start.distance_z(goal) < EPSILON_PLAN_XYZ) {

      // Change the goal and re-init
      goal_ = goal;
      init();

      return true;
    } else {
      return false;
    }
  }

  void LineSegment::init()
  {
    // Compute angle plan_ => goal_
    double angle_to_goal = atan2(goal_.y - plan_.y, goal_.x - plan_.x);

    // Drag force => thrust force => acceleration => feedforward
    drag_force_to_accel_xy(cxt_, goal_.yaw, cxt_.auv_xy_speed_ * cos(angle_to_goal),
                           cxt_.auv_xy_speed_ * sin(angle_to_goal), ff_.x, ff_.y);

    RCLCPP_INFO(logger_, "line: start (%g, %g), goal (%g, %g), ff (%g, %g)",
                plan_.x, plan_.y, goal_.x, goal_.y, ff_.x, ff_.y);
  }

  bool LineSegment::advance(double dt)
  {
    double distance_remaining = plan_.distance_xy(goal_);
    if (distance_remaining > EPSILON_PLAN_XYZ) {
      if (distance_remaining - deceleration_distance_xy(cxt_, goal_.yaw, twist_.x, twist_.y) < EPSILON_PLAN_XYZ) {
        // Decelerate
        ff_.x = ff_.y = 0;
      }

      // Compute acceleration due to drag
      double accel_drag_x, accel_drag_y;
      drag_force_to_accel_xy(cxt_, goal_.yaw, twist_.x, twist_.y, accel_drag_x, accel_drag_y);

      // Update velocity
      twist_.x += (ff_.x - accel_drag_x) * dt;
      twist_.y += (ff_.y - accel_drag_y) * dt;

      // Update plan
      plan_.x += twist_.x * dt;
      plan_.y += twist_.y * dt;

      return true;
    } else {
      finish();
      return false;
    }
  }

} // namespace orca_base