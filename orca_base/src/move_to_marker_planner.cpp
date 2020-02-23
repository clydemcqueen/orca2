#include "orca_base/move_to_marker_planner.hpp"

#include "orca_msgs/msg/control.hpp"

using namespace orca;

namespace orca_base
{

  //=====================================================================================
  // MoveToMarkerPlanner -- a recovery strategy
  //=====================================================================================

  MoveToMarkerPlanner::MoveToMarkerPlanner(const rclcpp::Logger &logger, const BaseContext &cxt,
                                           const ObservationStamped &start,
                                           PlannerStatus &status) :
    logger_{logger}, cxt_{cxt}, marker_id_{start.o.id},
    controller_{std::make_shared<ObservationController>(cxt_)}
  {
    // Start observation
    ObservationStamped plan = start;

    // Goal observation
    Observation goal;
    goal = start.o;

    // Rotate to face the marker
    goal.yaw = 0;
    segments_.push_back(std::make_shared<RotateToMarker>(logger_, cxt_, plan, goal));
    plan.o = goal;

    // Move toward the marker
    goal.distance = 1;
    segments_.push_back(std::make_shared<MoveToMarker>(logger_, cxt_, plan, goal));

    // Start
    segments_[0]->log_info();

    // Update status
    status.planner = orca_msgs::msg::Control::PLAN_RECOVERY_MTM;
    status.segments_total = segments_.size();
    status.segment_idx = 0;
    status.segment_info = ""; // TODO
  }

  bool MoveToMarkerPlanner::advance(const rclcpp::Duration &d, FPStamped &plan, const FPStamped &estimate,
                                    orca::Efforts &efforts, PlannerStatus &status)
  {
    // Advance the plan
    if (segments_[status.segment_idx]->advance(d)) {
      // Continue in this segment
    } else if (++status.segment_idx < segments_.size()) {
      // Move to next segment
      segments_[status.segment_idx]->log_info();
    } else {
      // Recovery action is complete
      return false;
    }

    // Share plan with caller (useful for diagnostics)
    plan = {};
    plan.fp.observations.push_back(segments_[status.segment_idx]->plan().o);

    // Run the PID controller(s) and calculate efforts
    // If marker was not observed, estimate.obs.id == NOT_A_MARKER, and calc() will ignore PID outputs
    Observation estimate_obs;
    estimate.fp.good_obs(marker_id_, estimate_obs);
    controller_->calc(d, segments_[status.segment_idx]->plan().o, cxt_.auv_z_target_, estimate_obs,
                      estimate.fp.pose.pose.z, segments_[status.segment_idx]->ff(), efforts);

    return true;
  }

} // namespace orca_base
