#include "orca_base/pose_planner.hpp"

#include "rclcpp/logging.hpp"

#include "orca_msgs/msg/control.hpp"

namespace orca_base
{

  //=====================================================================================
  // PosePlanner -- build a local plan to a target
  //=====================================================================================

  PosePlanner::PosePlanner(const rclcpp::Logger &logger, const AUVContext &cxt, const mw::PoseStamped &start,
                           mw::Target target, mw::Map map, bool keep_station, mw::MissionState &state) :
    LocalPlanner{LocalPlannerType::POSE_PLANNER},
    logger_{logger},
    cxt_{cxt},
    target_{std::move(target)},
    map_{std::move(map)},
    keep_station_{keep_station},
    controller_{std::make_shared<PoseController>(cxt_)}
  {
    // Future: build a plan that will keep markers in view at all times
    // For now, consider 2 cases:
    // 1. short plan: use a single segment to move from start to target
    // 2. long plan: find a series of waypoints (if possible), and always face the direction of motion

    // Start pose
    mw::PoseStamped plan = start;

    if (target.pose().position().distance_xy(start.pose().position()) < cxt_.pose_plan_max_short_plan_xy_) {
      RCLCPP_INFO_STREAM(logger_, "short plan " << target);

      add_pose_segment(plan, target.pose());

    } else {
      RCLCPP_INFO_STREAM(logger_, "long plan " << target);

      // Generate a series of waypoints to minimize dead reckoning
      std::vector<mw::Pose> waypoints;

      if (cxt_.pose_plan_waypoints_ &&
          map_.get_waypoints(cxt_.global_plan_target_z_, cxt_.pose_plan_max_dead_reckon_dist_,
                             start.pose(), target_.pose(), waypoints)) {
        RCLCPP_INFO(logger_, "... through %d waypoints", waypoints.size() - 1);
      } else {
        waypoints.push_back(target_.pose());
      }

      // Travel to each waypoint, breaking down z, yaw and xy phases
      for (auto &waypoint : waypoints) {
        // Ascend/descend to target z
        if (plan.pose().position().distance_z(waypoint.position()) > cxt_.pose_plan_epsilon_xyz_) {
          add_vertical_segment(plan, waypoint.z());
          add_keep_station_segment(plan, 1);
        } else {
          RCLCPP_INFO(logger_, "skip vertical");
        }

        if (plan.pose().position().distance_xy(waypoint.position()) > cxt_.pose_plan_epsilon_xyz_) {
          // Point in the direction of travel
          add_rotate_segment(plan, atan2(waypoint.y() - plan.pose().y(), waypoint.x() - plan.pose().x()));
          add_keep_station_segment(plan, 1);

          // Travel
          add_line_segment(plan, waypoint.x(), waypoint.y());
        } else {
          RCLCPP_INFO(logger_, "skip travel");
        }
      }

      // Always rotate to the target yaw
      add_rotate_segment(plan, target_.pose().yaw());
    }

    // Pause
    add_keep_station_segment(plan, 5);

#ifdef LOCAL_PATH
    // Create a path message to this target for diagnostics
    local_path_.header.frame_id = cxt_.map_frame_;
    local_path_.poses.clear();

    geometry_msgs::msg::PoseStamped pose_msg;
    for (auto &i : segments_) {
      pose_msg.pose = i->plan().fp.pose.pose.to_msg();
      local_path_.poses.push_back(pose_msg);
    }

    // Add last goal pose to path message
    pose_msg.pose = segments_.back()->goal().pose.pose.to_msg();
    local_path_.poses.push_back(pose_msg);
#endif

    if (keep_station_) {
      // Keep station at the last target
      add_keep_station_segment(plan, 1e6);
    }

    RCLCPP_INFO(logger_, "planned duration %g seconds", (plan.header().t() - start.header().t()).seconds());

    // Update state
    state.first_segment(orca_msgs::msg::MissionState::PLAN_LOCAL, segments_.size(),
                        segments_[0]->to_str(), segments_[0]->type());
    RCLCPP_INFO_STREAM(logger_, "segment 1 of " << segments_.size() << ", " << state.segment_info());

    state.set_plan(segments_[state.segment_idx()]->plan(), map_);
    state.twist() = segments_[state.segment_idx()]->twist();
  }

  void PosePlanner::add_keep_station_segment(mw::PoseStamped &plan, double seconds)
  {
    segments_.push_back(std::make_shared<Pause>(cxt_, plan, rclcpp::Duration::from_seconds(seconds)));
  }

  void PosePlanner::add_vertical_segment(mw::PoseStamped &plan, double z)
  {
    segments_.push_back(Trap2::make_vertical(cxt_, plan, z));
  }

  void PosePlanner::add_rotate_segment(mw::PoseStamped &plan, double yaw)
  {
    segments_.push_back(Trap2::make_rotate(cxt_, plan, yaw));
  }

  void PosePlanner::add_line_segment(mw::PoseStamped &plan, double x, double y)
  {
    segments_.push_back(Trap2::make_line(cxt_, plan, x, y));
  }

  void PosePlanner::add_pose_segment(mw::PoseStamped &plan, const mw::Pose &goal)
  {
    segments_.push_back(Trap2::make_pose(cxt_, plan, goal));
  }

  bool PosePlanner::advance(const rclcpp::Duration &d, const mw::FiducialPoseStamped &estimate, mw::Efforts &efforts,
                            mw::MissionState &state)
  {
    // Update the plan
    if (segments_[state.segment_idx()]->advance(d)) {
      // Continue in this segment
    } else if (state.segment_idx() + 1 < segments_.size()) {
      // Move to the next segment
      state.next_segment(segments_[state.segment_idx() + 1]->to_str(), segments_[state.segment_idx() + 1]->type());
      RCLCPP_INFO_STREAM(logger_, "segment " << state.segment_idx() + 1 << " of " << segments_.size() << ", " << state.segment_info());
    } else {
      // Local plan is complete
      return false;
    }

    state.set_plan(segments_[state.segment_idx()]->plan(), map_);
    state.twist() = segments_[state.segment_idx()]->twist();

    // Run PID controller and calculate efforts
    controller_->calc(d, state.plan().fp(), estimate.fp(), segments_[state.segment_idx()]->ff(), efforts);

    return true;
  }

} // namespace orca_base
