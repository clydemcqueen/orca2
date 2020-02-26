#include "orca_base/planner_common.hpp"

#include <iomanip>
#include <utility>

#include "orca_msgs/msg/control.hpp"

namespace orca_base
{

  void PlannerStatus::first_target(int _targets_total, int _target_marker_id)
  {
    targets_total = _targets_total;
    target_idx = 0;
    target_marker_id = _target_marker_id;

    planner = orca_msgs::msg::Control::PLAN_NONE;
    local_plan_idx = -1; // Start at -1, first_segment will increment

    segments_total = 0;
    segment_idx = 0;
    segment_info = "";
    segment_type = orca_msgs::msg::Control::SEGMENT_NONE;

    pose = {};
    twist = {};
  }

  void PlannerStatus::next_target(int next_marker_id)
  {
    target_marker_id = next_marker_id;

    planner = orca_msgs::msg::Control::PLAN_NONE;
    local_plan_idx = -1; // Start at -1, first_segment will increment
  }

  void
  PlannerStatus::first_segment(uint8_t _planner, int _segments_total, std::string _segment_info, uint8_t _segment_type)
  {
    planner = _planner;
    local_plan_idx++;

    segments_total = _segments_total;
    segment_idx = 0;
    segment_info = std::move(_segment_info);
    segment_type = _segment_type;
  }

  void PlannerStatus::next_segment(std::string _segment_info, uint8_t _segment_type)
  {
    segment_info = std::move(_segment_info);
    segment_type = _segment_type;
  }

  std::ostream &operator<<(std::ostream &os, Target const &target)
  {
    os << std::fixed << std::setprecision(2)
       << "{marker: " << target.marker_id << ", pose: " << target.fp.pose.pose << "}";
  }

}