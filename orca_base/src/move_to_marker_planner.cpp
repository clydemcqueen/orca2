#include "orca_base/move_to_marker_planner.hpp"

#include "rclcpp/logging.hpp"

#include "orca_msgs/msg/control.hpp"

using namespace orca;

namespace orca_base
{

  //=====================================================================================
  // MoveToMarkerPlanner -- a recovery strategy
  //=====================================================================================

  MoveToMarkerPlanner::MoveToMarkerPlanner(const rclcpp::Logger &logger, const AUVContext &cxt,
                                           const ObservationStamped &start,
                                           PlannerStatus &status) :
    LocalPlanner{LocalPlannerType::MTM_PLANNER}, logger_{logger}, cxt_{cxt}, marker_id_{start.o.id},
    controller_{std::make_shared<ObservationController>(cxt_)}
  {
    // Start observation
    ObservationStamped plan = start;

    // Goal observation
    Observation goal;
    goal = start.o;

    // Rotate to face the marker
    goal.bearing = 0;
    segments_.push_back(std::make_shared<RotateToMarker>(cxt_, plan, goal));
    plan.o = goal;

    // Move toward the marker
    goal.distance = cxt_.mtm_plan_target_dist_;
    segments_.push_back(std::make_shared<MoveToMarker>(cxt_, plan, goal));

    RCLCPP_INFO(logger_, "segment 1 of %d", segments_.size());

    // Update status
    status.first_segment(orca_msgs::msg::Control::PLAN_RECOVERY_MTM, segments_.size(),
                         segments_[0]->to_str(), segments_[0]->type());
    RCLCPP_INFO(logger_, status.segment_info);

    // Set initial pose and twist
    status.pose = {};
    status.pose.fp.observations.push_back(segments_[0]->plan().o);
    status.twist = {}; // Don't know yaw in the world frame, so can't convert TwistBody into Twist
  }

  bool MoveToMarkerPlanner::advance(const rclcpp::Duration &d, const FPStamped &estimate, orca::Efforts &efforts,
                                    PlannerStatus &status)
  {
    // Advance the plan
    if (segments_[status.segment_idx]->advance(d)) {
      // Continue in this segment
    } else if (++status.segment_idx < segments_.size()) {
      // Move to next segment
      RCLCPP_INFO(logger_, "segment %d of %d", status.segment_idx + 1, segments_.size());

      // Update status
      status.next_segment(segments_[status.segment_idx]->to_str(), segments_[status.segment_idx]->type());
      RCLCPP_INFO(logger_, status.segment_info);
    } else {
      // Recovery action is complete
      return false;
    }

    // Update pose and twist
    status.pose = {};
    status.pose.fp.observations.push_back(segments_[status.segment_idx]->plan().o);
    status.twist = {}; // Don't know yaw in the world frame, so can't convert TwistBody into Twist

    // Run the PID controller(s) and calculate efforts
    // If marker was not observed, estimate.obs.id == NOT_A_MARKER, and calc() will ignore PID outputs
    Observation estimate_obs;
    estimate.fp.get_observation(marker_id_, estimate_obs);
    controller_->calc(d, segments_[status.segment_idx]->plan().o, cxt_.global_plan_target_z_, estimate_obs,
                      estimate.fp.pose.pose.z, segments_[status.segment_idx]->ff(), efforts);

    return true;
  }

} // namespace orca_base
