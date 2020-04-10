#include "orca_base/move_to_marker_planner.hpp"

#include "rclcpp/logging.hpp"

#include "orca_msgs/msg/control.hpp"

namespace orca_base
{

  //=====================================================================================
  // MoveToMarkerPlanner -- a recovery strategy
  //=====================================================================================

  MoveToMarkerPlanner::MoveToMarkerPlanner(const rclcpp::Logger &logger, const AUVContext &cxt,
                                           const mw::PolarObservationStamped &start,
                                           mw::MissionState &state) :
    LocalPlanner{LocalPlannerType::MTM_PLANNER}, logger_{logger}, cxt_{cxt}, marker_id_{start.observation().id()},
    controller_{std::make_shared<ObservationController>(cxt_)}
  {
    // Start observation
    mw::PolarObservationStamped plan = start;

    // Goal observation
    mw::PolarObservation goal = start.observation();

    // Rotate to face the marker
    goal.bearing() = 0;
    segments_.push_back(std::make_shared<RotateToMarker>(cxt_, plan, goal));
    plan.observation() = goal;

    // Move toward the marker
    goal.distance() = cxt_.mtm_plan_target_dist_;
    segments_.push_back(std::make_shared<MoveToMarker>(cxt_, plan, goal));

    // Update state
    state.first_segment(orca_msgs::msg::MissionState::PLAN_RECOVERY_MTM, segments_.size(),
                        segments_[0]->to_str(), segments_[0]->type());
    RCLCPP_INFO_STREAM(logger_, "segment 1 of " << segments_.size() << ", " << state.segment_info());

    state.set_plan(segments_[state.segment_idx()]->plan());
    state.twist() = {}; // Don't know yaw in the world frame, so can't convert TwistBody into Twist
  }

  bool MoveToMarkerPlanner::advance(const rclcpp::Duration &d, const mw::FiducialPoseStamped &estimate,
                                    mw::Efforts &efforts, mw::MissionState &state)
  {
    // Advance the plan
    if (segments_[state.segment_idx()]->advance(d)) {
      // Continue in this segment
    } else if (state.segment_idx() + 1 < segments_.size()) {
      // Move to next segment
      state.next_segment(segments_[state.segment_idx() + 1]->to_str(), segments_[state.segment_idx() + 1]->type());
      RCLCPP_INFO_STREAM(logger_, "segment " << state.segment_idx() + 1 << " of " << segments_.size() << ", "
                                             << state.segment_info());
    } else {
      // Recovery action is complete
      return false;
    }

    state.set_plan(segments_[state.segment_idx()]->plan());
    state.twist() = {}; // Don't know yaw in the world frame, so can't convert TwistBody into Twist

    // Run the PID controller(s) and calculate efforts
    // If marker was not observed, estimate.obs.id == NOT_A_MARKER, and calc() will ignore PID outputs
    mw::PolarObservation estimate_obs = estimate.fp().observations().get_polar(marker_id_);
    controller_->calc(d, segments_[state.segment_idx()]->plan().observation(), cxt_.global_plan_target_z_, estimate_obs,
                      estimate.fp().pose().pose().z(), segments_[state.segment_idx()]->ff(), efforts);

    return true;
  }

} // namespace orca_base
