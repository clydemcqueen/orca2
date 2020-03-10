#include "orca_base/trap2_segment.hpp"

#include <iomanip>
#include <iostream>

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

  //=====================================================================================
  // FastPlan
  //=====================================================================================

  struct FastPlan
  {
    double t_ramp;    // Time (seconds) for ramp up (phase 1) and ramp down (phase 3)
    double t_run;     // Time (seconds) for run (phase 2)

    /**
     * Find the fastest trapezoidal velocity plan, given max acceleration and velocity
     * Total time = 2 * t_ramp + t_run
     *
     * @param angle True if d is an angle
     * @param d The distance to cover
     * @param a_max Constraint: maximum acceleration
     * @param v_max Constraint: maximum velocity
     */
    FastPlan(bool angle, double d, double a_max, double v_max)
    {
      assert(a_max > 0);
      assert(v_max > 0);

      if (angle) {
        d = orca::norm_angle(d);
      }

      d = std::abs(d);

      t_ramp = v_max / a_max;
      auto d_run = d - a_max * t_ramp * t_ramp;
      if (angle) {
        d_run = orca::norm_angle(d_run);
      }
      t_run = d_run / v_max;

      if (t_run < 0) {
        // Distance too short, will not hit v_max
        t_ramp = sqrt(d / a_max);
        t_run = 0;
      }
    }
  };

  std::ostream &operator<<(std::ostream &os, FastPlan const &p)
  {
    return os << std::fixed << std::setprecision(3) << "{t_ramp: " << p.t_ramp << ", t_run: " << p.t_run << "}";
  }

  //=====================================================================================
  // SyncPlan
  //=====================================================================================

  struct SyncPlan
  {
    double a;         // Acceleration for ramp
    double v;         // Velocity for run
    double d_ramp;    // Distance covered during ramp
    double d_run;     // Distance covered during run

    /**
     * Find a trapezoidal velocity plan with a given t_ramp and t_run
     *
     * @param angle True if d is an angle
     * @param d The distance to cover
     * @param t_ramp Ramp time
     * @param t_run Run time
     */
    SyncPlan(bool angle, double d, double t_ramp, double t_run)
    {
      if (angle) {
        d = orca::norm_angle(d);
      }

      a = d / (t_ramp * (t_ramp + t_run));
      v = a * t_ramp;
      d_ramp = 0.5 * v * t_ramp;
      d_run = v * t_run;
    }
  };

  std::ostream &operator<<(std::ostream &os, SyncPlan const &p)
  {
    return os << std::fixed << std::setprecision(3)
              << "{a: " << p.a << ", v: " << p.v << ", d_ramp: " << p.d_ramp << ", d_run: " << p.d_run << "}";
  }

  //=====================================================================================
  // plan_sync
  //=====================================================================================

  void plan_sync(const AUVContext &cxt, const orca::PoseStamped &p0,
                 orca::PoseStamped &p1, orca::PoseStamped &p2, orca::PoseStamped &p3,
                 orca::Acceleration &a0, orca::Twist &v1)
  {
    /* Notes on motion in the xy plane:
     *
     * If we move in x and y separately we end up with higher 2D velocity for diagonal motion
     * (e.g., x+5, y+5) than for motion along an axis (e.g., x+0, y+5). Avoid this by computing
     * xy motion together.
     *
     * We do still have cases where we end up with higher 3D velocity for diagonal motion
     * (e.g., x+5, y+5, z+5). This is OK because xy and z motion use different thrusters.
     *
     * We are left with the case where xy motion and yaw motion, which use the same thrusters,
     * may combine to saturate the thrusters. For now we avoid this by picking the right values
     * for auv_xy_accel, auv_xy_velo, auv_yaw_accel, auv_yaw_velo, and xy_gain.
     */

    // Create the fastest plans
    double dxy = p3.pose.distance_xy(p0.pose);
    FastPlan xy_fast(false, dxy, cxt.auv_xy_accel_, cxt.auv_xy_velo_);
    FastPlan z_fast(false, p3.pose.z - p0.pose.z, cxt.auv_z_accel_, cxt.auv_z_velo_);
    FastPlan yaw_fast(true, p3.pose.yaw - p0.pose.yaw, cxt.auv_yaw_accel_, cxt.auv_yaw_velo_);

    // Find the longest ramp and run times
    double t_ramp = xy_fast.t_ramp;
    if (t_ramp < z_fast.t_ramp) t_ramp = z_fast.t_ramp;
    if (t_ramp < yaw_fast.t_ramp) t_ramp = yaw_fast.t_ramp;

    double t_run = xy_fast.t_run;
    if (t_run < z_fast.t_run) t_run = z_fast.t_run;
    if (t_run < yaw_fast.t_run) t_run = yaw_fast.t_run;

    // Create plans where the phases are synchronized
    SyncPlan xy_sync(false, dxy, t_ramp, t_run);
    SyncPlan z_sync(false, p3.pose.z - p0.pose.z, t_ramp, t_run);
    SyncPlan yaw_sync(true, p3.pose.yaw - p0.pose.yaw, t_ramp, t_run);

    // Save results
    p1.t = p0.t + rclcpp::Duration::from_seconds(t_ramp);
    p2.t = p1.t + rclcpp::Duration::from_seconds(t_run);
    p3.t = p2.t + rclcpp::Duration::from_seconds(t_ramp);

    auto angle_to_goal = atan2(p3.pose.y - p0.pose.y, p3.pose.x - p0.pose.x);
    auto xf = cos(angle_to_goal);   // x fraction of xy motion
    auto yf = sin(angle_to_goal);   // y fraction of xy motion

    p1.pose = p0.pose + orca::Pose(xf * xy_sync.d_ramp, yf * xy_sync.d_ramp, z_sync.d_ramp, yaw_sync.d_ramp);
    p2.pose = p1.pose + orca::Pose(xf * xy_sync.d_run, yf * xy_sync.d_run, z_sync.d_run, yaw_sync.d_run);

    a0.x = xf * xy_sync.a;
    a0.y = yf * xy_sync.a;
    a0.z = z_sync.a;
    a0.yaw = yaw_sync.a;

    v1.x = xf * xy_sync.v;
    v1.y = yf * xy_sync.v;
    v1.z = z_sync.v;
    v1.yaw = yaw_sync.v;
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
