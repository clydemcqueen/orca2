#include "orca_base/segment.hpp"

#include <utility>

using namespace orca;

namespace orca_base
{

  //=====================================================================================
  // Utilities
  //=====================================================================================

  // Run some numerical approximations
  constexpr int NUM_MAX_TICKS = 100;    // Max number of ticks
  constexpr double NUM_DT = 0.1;        // Time step for each tick
  constexpr double END_VELO_XY = 0.1;   // When xy velo is < this, end the simulation
  constexpr double END_VELO_Z = 0.1;    // When z velo is < this, end the simulation
  constexpr double END_VELO_YAW = 0.1;  // When yaw velo is < this, end the simulation

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
  // PoseSegment
  //=====================================================================================

  PoseSegment::PoseSegment(const rclcpp::Logger &logger, const BaseContext &cxt, const orca::FPStamped &start,
                           const orca::FP &goal) :
    PoseSegmentBase{logger, cxt, start, goal}
  {
    // Calc target velocity
    if (start.fp.distance_xy(goal) > EPSILON_PLAN_XYZ) {
      double angle_to_goal = atan2(goal_.pose.pose.y - plan_.fp.pose.pose.y, goal_.pose.pose.x - plan_.fp.pose.pose.x);
      target_twist_.x = cxt_.auv_xy_speed_ * cos(angle_to_goal);
      target_twist_.y = cxt_.auv_xy_speed_ * sin(angle_to_goal);
    }

    if (start.fp.distance_z(goal) > EPSILON_PLAN_XYZ) {
      target_twist_.z = goal.pose.pose.z > start.fp.pose.pose.z ? cxt.auv_z_speed_ : -cxt.auv_z_speed_;
    }

    if (start.fp.distance_yaw(goal) > EPSILON_PLAN_YAW) {
      target_twist_.yaw = norm_angle(goal.pose.pose.yaw - start.fp.pose.pose.yaw) > 0 ?
                          cxt.auv_yaw_speed_ : -cxt.auv_yaw_speed_;
    }

    // std::cout << "target_twist_: " << target_twist_ << std::endl;

    // Calc feedforward for z and yaw, for x and y feedforward will vary with yaw
    // Drag force (torque) => thrust force (torque) => acceleration == feedforward
    ff_.yaw = Model::torque_to_accel_yaw(-cxt.model_.drag_torque_yaw(target_twist_.yaw));
    ff_.z += Model::force_to_accel(-cxt.model_.drag_force_z(target_twist_.z));
  }

  bool PoseSegment::advance(const rclcpp::Duration &d)
  {
    // Update plan
    plan_.t = plan_.t + d;

    // End when goal ~= plan
    double remaining_xy = plan_.fp.distance_xy(goal_);
    double remaining_z = plan_.fp.distance_z(goal_);
    double remaining_yaw = plan_.fp.distance_yaw(goal_);

    if (remaining_xy < EPSILON_PLAN_XYZ && remaining_z < EPSILON_PLAN_XYZ && remaining_yaw < EPSILON_PLAN_YAW) {
      return false;
    } else {

      // Glide
      if (!glide_xy_ &&
          remaining_xy - deceleration_distance_xy(cxt_, goal_.pose.pose.yaw, twist_.x, twist_.y) < EPSILON_PLAN_XYZ) {
        RCLCPP_INFO(logger_, "glide xy");
        glide_xy_ = true;
        ff_.x = ff_.y = 0;
      }

      if (!glide_z_ && remaining_z - deceleration_distance_z(cxt_, twist_.z) < EPSILON_PLAN_XYZ) {
        RCLCPP_INFO(logger_, "glide z");
        glide_z_ = true;
        ff_.z = cxt_.model_.hover_accel_z();
      }

      if (!glide_yaw_ && remaining_yaw - deceleration_distance_yaw(cxt_, twist_.yaw) < EPSILON_PLAN_YAW) {
        RCLCPP_INFO(logger_, "glide yaw");
        glide_yaw_ = true;
        ff_.yaw = 0;
      }

      // Update xy feedforward
      if (!glide_xy_) {
        drag_force_to_accel_xy(cxt_, plan_.fp.pose.pose.yaw, target_twist_.x, target_twist_.y, ff_.x, ff_.y);
      }

      // std::cout << "ff_: " << ff_ << std::endl;

      // Calc acceleration due to drag
      Acceleration drag_;
      drag_force_to_accel_xy(cxt_, plan_.fp.pose.pose.yaw, twist_.x, twist_.y, drag_.x, drag_.y);
      drag_.yaw = Model::torque_to_accel_yaw(-cxt_.model_.drag_torque_yaw(twist_.yaw));
      drag_.z = Model::force_to_accel(-cxt_.model_.drag_force_z(twist_.z));

      // std::cout << "drag_: " << drag_ << std::endl;

      auto dt = d.seconds();

      // Update velocity
      twist_.x += (ff_.x - drag_.x) * dt;
      twist_.y += (ff_.y - drag_.y) * dt;
      twist_.z += (ff_.z - cxt_.model_.hover_accel_z() - drag_.z) * dt;
      twist_.yaw += (ff_.yaw - drag_.yaw) * dt;

      // std::cout << "twist_: " << twist_ << std::endl;

      // Update plan
      plan_.fp.pose.pose.x += twist_.x * dt;
      plan_.fp.pose.pose.y += twist_.y * dt;
      plan_.fp.pose.pose.z += twist_.z * dt;
      plan_.fp.pose.pose.yaw = norm_angle(plan_.fp.pose.pose.yaw + twist_.yaw * dt);

      return true;
    }
  }

  void PoseSegment::log_info()
  {
    RCLCPP_INFO_STREAM(logger_, "pose segment start: " << plan_ << ", goal: " << goal_);
  }

  std::shared_ptr<PoseSegment>
  PoseSegment::make_vertical(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FPStamped &plan, double z)
  {
    FP goal = plan.fp;
    goal.pose.pose.z = z;
    auto result = std::make_shared<PoseSegment>(logger, cxt, plan, goal);
    plan.fp = goal;
    return result;
  }

  std::shared_ptr<PoseSegment>
  PoseSegment::make_rotate(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FPStamped &plan, double yaw)
  {
    FP goal = plan.fp;
    goal.pose.pose.yaw = yaw;
    auto result = std::make_shared<PoseSegment>(logger, cxt, plan, goal);
    plan.fp = goal;
    return result;
  }

  std::shared_ptr<PoseSegment>
  PoseSegment::make_line(const rclcpp::Logger &logger, const BaseContext &cxt, orca::FPStamped &plan, double x,
                         double y)
  {
    FP goal = plan.fp;
    goal.pose.pose.x = x;
    goal.pose.pose.y = y;
    auto result = std::make_shared<PoseSegment>(logger, cxt, plan, goal);
    plan.fp = goal;
    return result;
  }

  std::shared_ptr<PoseSegment>
  PoseSegment::make_pose(const rclcpp::Logger &logger, const BaseContext &cxt, FPStamped &plan, const orca::FP &goal)
  {
    auto result = std::make_shared<PoseSegment>(logger, cxt, plan, goal);
    plan.fp = goal;
    return result;
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

  bool MoveToMarkerSegment::advance(const rclcpp::Duration &d)
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

  bool RotateToMarkerSegment::advance(const rclcpp::Duration &d)
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