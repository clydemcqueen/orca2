#ifndef ORCA_SHARED_MW_MISSION_STATUS_HPP
#define ORCA_SHARED_MW_MISSION_STATUS_HPP

#include "orca_msgs/msg/mission_state.hpp"
#include "orca_shared/mw/fiducial_pose_stamped.hpp"
#include "orca_shared/mw/polar_observation_stamped.hpp"
#include "orca_shared/mw/twist.hpp"

namespace mw
{

  class MissionState
  {
    orca_msgs::msg::MissionState msg_;
    FiducialPoseStamped plan_;
    Twist twist_;

  public:

    MissionState() = default;

    explicit MissionState(const orca_msgs::msg::MissionState &msg) :
      msg_{msg}
    {
      plan_ = FiducialPoseStamped{msg_.pose};
      twist_ = Twist{msg_.twist};
    }

    MissionState(const Observer &observer, const std::string &_mission_info, const int &_targets_total, const int &_target_marker_id) :
      plan_{observer}
    {
      msg_.mission_info = _mission_info;
      msg_.targets_total = _targets_total;
      msg_.target_marker_id = _target_marker_id;

      msg_.planner = orca_msgs::msg::MissionState::PLAN_NONE;
      msg_.segment_type = orca_msgs::msg::MissionState::SEGMENT_NONE;

      msg_.local_plan_idx = -1; // Start at -1, first_segment will increment
    }

    void next_target(int next_marker_id)
    {
      msg_.target_marker_id = next_marker_id;
      msg_.planner = orca_msgs::msg::MissionState::PLAN_NONE;

      msg_.target_idx++;
      msg_.local_plan_idx = -1; // Start at -1, first_segment will increment
    }

    void first_segment(uint8_t _planner, int _segments_total, std::string _segment_info, uint8_t _segment_type)
    {
      msg_.planner = _planner;
      msg_.segments_total = _segments_total;
      msg_.segment_info = std::move(_segment_info);
      msg_.segment_type = _segment_type;

      msg_.local_plan_idx++;
      msg_.segment_idx = 0;
    }

    void next_segment(std::string _segment_info, uint8_t _segment_type)
    {
      msg_.segment_info = std::move(_segment_info);
      msg_.segment_type = _segment_type;

      msg_.segment_idx++;
    }

    orca_msgs::msg::MissionState msg() const
    {
      auto msg = msg_;
      msg.pose = plan_.msg();
      msg.twist = twist_.msg();
      return msg;
    }

    const std::string &mission_info() const
    {
      return msg_.mission_info;
    }

    int targets_total() const
    {
      return msg_.targets_total;
    }

    int target_idx() const
    {
      return msg_.target_idx;
    }

    int target_marker_id() const
    {
      return msg_.target_marker_id;
    }

    uint8_t planner() const
    {
      return msg_.planner;
    }

    int local_plan_idx() const
    {
      return msg_.local_plan_idx;
    }

    int segments_total() const
    {
      return msg_.segments_total;
    }

    int segment_idx() const
    {
      return msg_.segment_idx;
    }

    const std::string &segment_info() const
    {
      return msg_.segment_info;
    }

    uint8_t segment_type() const
    {
      return msg_.segment_type;
    }

    uint8_t phase() const
    {
      return msg_.phase;
    }

    uint8_t &phase()
    {
      return msg_.phase;
    }

    const FiducialPoseStamped &plan() const
    {
      return plan_;
    }

    const Twist &twist() const
    {
      return twist_;
    }

    Twist &twist()
    {
      return twist_;
    }

    void set_plan(const PoseStamped &plan, const Map &map)
    {
      plan_.header() = plan.header();
      plan_.fp().pose().pose() = plan.pose();
      plan_.fp().pose().covariance() = {};
      plan_.fp().predict_observations(map);
    }

    void set_plan(const PolarObservationStamped &polar_observation)
    {
      plan_.header() = polar_observation.header();
      plan_.fp().observations().clear();
      plan_.fp().observations().add_polar(polar_observation.observation());
    }

    bool operator==(const MissionState &that) const
    {
      return msg_ == that.msg_ &&
             plan_ == that.plan_ &&
             twist_ == that.twist_;
    }

    bool operator!=(const MissionState &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const MissionState &v);
  };

}

#endif //ORCA_SHARED_MW_MISSION_STATUS_HPP
