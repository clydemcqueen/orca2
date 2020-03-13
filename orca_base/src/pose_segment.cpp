#include "orca_base/pose_segment.hpp"

#include <iomanip>
#include <iostream>

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

  //=====================================================================================
  // PoseSegmentBase
  //=====================================================================================

  PoseSegmentBase::PoseSegmentBase(const AUVContext &cxt, uint8_t type, FPStamped start, FP goal) :
    SegmentBase{cxt, type},
    plan_{std::move(start)},
    goal_{std::move(goal)}
  {
    // Default ff includes acceleration to counteract buoyancy
    ff_ = orca::Acceleration{0, 0, cxt.model_.hover_accel_z(), 0};
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
  // Trap2
  //=====================================================================================

  Trap2::Trap2(const AUVContext &cxt, uint8_t type, const FPStamped &start, const FP &goal) :
    PoseSegmentBase{cxt, type, start, goal}
  {
    p0_.t = start.t;
    p0_.pose = start.fp.pose.pose;
    // plan_sync will set p3_.t
    p3_.pose = goal.pose.pose;

    // Create a synchronized motion plan
    plan_sync(cxt_, p0_, p1_, p2_, p3_, a0_, v1_);
  }

  rclcpp::Duration Trap2::duration() const
  {
    return p3_.t - p0_.t;
  }

  std::string Trap2::to_str()
  {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3)
       << type_name() << ", accel dt: " << (p1_.t - p0_.t).seconds()
       << ", run dt: " << (p2_.t - p1_.t).seconds()
       << ", decel dt: " << (p3_.t - p2_.t).seconds();
    return ss.str();
  }

  bool Trap2::advance(const rclcpp::Duration &d)
  {
    plan_.t = plan_.t + d;

    // Update total acceleration, twist_ and pose
    orca::Acceleration accel;
    if (plan_.t >= p0_.t && plan_.t < p1_.t) {
      // Phase 1: accelerate
      accel = a0_;
      twist_ = orca::Twist{}.project(plan_.t - p0_.t, accel);
      plan_.fp.pose.pose = p0_.pose.project(plan_.t - p0_.t, {}, accel);
    } else if (plan_.t < p2_.t) {
      // Phase 2: run at constant velocity
      accel = {};
      twist_ = v1_;
      plan_.fp.pose.pose = p1_.pose.project(plan_.t - p1_.t, v1_, accel);
    } else if (plan_.t < p3_.t) {
      // Phase 3: decelerate
      accel = -a0_;
      twist_ = v1_.project(plan_.t - p2_.t, accel);
      plan_.fp.pose.pose = p2_.pose.project(plan_.t - p2_.t, v1_, accel);
    } else {
      // Outside the plan
      return false;
    }

    // Compute acceleration due to drag
    // Drag must be computed in the body frame
    double twist_forward, twist_strafe;
    orca::rotate_frame(twist_.x, twist_.y, plan_.fp.pose.pose.yaw, twist_forward, twist_strafe);
    auto drag_forward = cxt_.model_.drag_accel_f(twist_forward);
    auto drag_strafe = cxt_.model_.drag_accel_s(twist_strafe);
    orca::Acceleration drag;
    orca::rotate_frame(drag_forward, drag_strafe, -plan_.fp.pose.pose.yaw, drag.x, drag.y);

    // Continue with z and yaw drag
    drag.z = cxt_.model_.drag_accel(twist_.z, cxt_.model_.drag_const_z());
    drag.yaw = cxt_.model_.drag_accel_yaw(twist_.yaw);

    // Feedforward (acceleration due to thrust) is total acceleration minus drag
    ff_ = accel - drag;

    // Add hover to feedforward
    ff_.z += cxt_.model_.hover_accel_z();

    return true;
  }

  std::shared_ptr<Trap2>
  Trap2::make_vertical(const AUVContext &cxt, FPStamped &plan, double z)
  {
    FP goal = plan.fp;
    goal.pose.pose.z = z;
    auto result = std::make_shared<Trap2>(cxt, orca_msgs::msg::Control::POSE_VERTICAL, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  std::shared_ptr<Trap2>
  Trap2::make_rotate(const AUVContext &cxt, FPStamped &plan, double yaw)
  {
    FP goal = plan.fp;
    goal.pose.pose.yaw = yaw;
    auto result = std::make_shared<Trap2>(cxt, orca_msgs::msg::Control::POSE_ROTATE, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  std::shared_ptr<Trap2>
  Trap2::make_line(const AUVContext &cxt, FPStamped &plan, double x, double y)
  {
    FP goal = plan.fp;
    goal.pose.pose.x = x;
    goal.pose.pose.y = y;
    auto result = std::make_shared<Trap2>(cxt, orca_msgs::msg::Control::POSE_LINE, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

  std::shared_ptr<Trap2>
  Trap2::make_pose(const AUVContext &cxt, FPStamped &plan, const FP &goal)
  {
    auto result = std::make_shared<Trap2>(cxt, orca_msgs::msg::Control::POSE_COMBO, plan, goal);
    plan.fp = goal;
    plan.t = plan.t + result->duration();
    return result;
  }

} // namespace orca_base
