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

  std::string SegmentBase::type_name() const
  {
    if (type_ == orca_msgs::msg::MissionState::PAUSE) {
      return "pause";
    } else if (type_ == orca_msgs::msg::MissionState::POSE_VERTICAL) {
      return "pose_vertical";
    } else if (type_ == orca_msgs::msg::MissionState::POSE_ROTATE) {
      return "pose_rotate";
    } else if (type_ == orca_msgs::msg::MissionState::POSE_LINE) {
      return "pose_line";
    } else if (type_ == orca_msgs::msg::MissionState::POSE_COMBO) {
      return "pose_combo";
    } else if (type_ == orca_msgs::msg::MissionState::OBS_RTM) {
      return "obs_rtm";
    } else if (type_ == orca_msgs::msg::MissionState::OBS_MTM) {
      return "obs_mtm";
    } else {
      return "no_segment";
    }
  }

  //=====================================================================================
  // FastPlan
  //=====================================================================================

  FastPlan::FastPlan(bool angle, double d, double a_max, double v_max)
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

  std::ostream &operator<<(std::ostream &os, FastPlan const &p)
  {
    return os << std::fixed << std::setprecision(3) << "{t_ramp: " << p.t_ramp << ", t_run: " << p.t_run << "}";
  }

  //=====================================================================================
  // SyncPlan
  //=====================================================================================

  SyncPlan::SyncPlan(bool angle, double d, double t_ramp, double t_run)
  {
    if (angle) {
      d = orca::norm_angle(d);
    }

    a = d / (t_ramp * (t_ramp + t_run));
    v = a * t_ramp;
    d_ramp = 0.5 * v * t_ramp;
    d_run = v * t_run;
  }

  std::ostream &operator<<(std::ostream &os, SyncPlan const &p)
  {
    return os << std::fixed << std::setprecision(3)
              << "{a: " << p.a << ", v: " << p.v << ", d_ramp: " << p.d_ramp << ", d_run: " << p.d_run << "}";
  }

} // namespace orca_base