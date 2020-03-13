#include "orca_base/segment_common.hpp"

#include <iomanip>

#include "orca_msgs/msg/control.hpp"
#include "orca_shared/util.hpp"

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
    } else {
      return "no_segment";
    }
  }

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

} // namespace orca_base