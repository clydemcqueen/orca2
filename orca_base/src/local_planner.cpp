#include "orca_base/local_planner.hpp"

#include "rclcpp/logging.hpp"

#include "orca_msgs/msg/control.hpp"

using namespace orca;

namespace orca_base
{

  //=====================================================================================
  // LocalPlanner -- build a local plan to a target
  //=====================================================================================

  LocalPlanner::LocalPlanner(const rclcpp::Logger &logger, const AUVContext &cxt, const FPStamped &start,
                             Target target, Map map,
                             bool keep_station, PlannerStatus &status) :
    logger_{logger}, cxt_{cxt}, target_{std::move(target)}, map_{std::move(map)}, keep_station_{keep_station},
    controller_{std::make_shared<PoseController>(cxt_)}
  {
    // Future: build a plan that will keep markers in view at all times
    // For now, consider 2 cases:
    // 1. short plan: use a single segment to move from start to target
    // 2. long plan: find a series of waypoints (if possible), and always face the direction of motion

    // Start pose
    FPStamped plan = start;

    if (target.fp.distance_xy(start.fp) < cxt_.planner_max_short_plan_xy_) {
      RCLCPP_INFO_STREAM(logger_, "short plan: " << target);

      add_pose_segment(plan, target.fp);

    } else {
      RCLCPP_INFO_STREAM(logger_, "long plan: " << target);

      // Generate a series of waypoints to minimize dead reckoning
      std::vector<Pose> waypoints;

      if (cxt_.planner_look_for_waypoints_ && map_.get_waypoints(start.fp.pose.pose, target_.fp.pose.pose, waypoints)) {
        RCLCPP_INFO(logger_, "... through %d waypoints", waypoints.size() - 1);
      } else {
        waypoints.push_back(target_.fp.pose.pose);
      }

      // Travel to each waypoint, breaking down z, yaw and xy phases
      for (auto &waypoint : waypoints) {
        // Ascend/descend to target z
        add_vertical_segment(plan, waypoint.z);

        if (plan.fp.pose.pose.distance_xy(waypoint.x, waypoint.y) > cxt_.planner_epsilon_xyz_) {
          // Point in the direction of travel
          add_rotate_segment(plan, atan2(waypoint.y - plan.fp.pose.pose.y, waypoint.x - plan.fp.pose.pose.x));

          // Travel
          add_line_segment(plan, waypoint.x, waypoint.y);
        } else {
          RCLCPP_INFO(logger_, "skip travel");
        }
      }

      // Always rotate to the target yaw
      add_rotate_segment(plan, target_.fp.pose.pose.yaw);
    }

    // Pause
    add_keep_station_segment(plan, 5);

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

    if (keep_station_) {
      // Keep station at the last target
      add_keep_station_segment(plan, 1e6);
    }

    RCLCPP_INFO(logger_, "planned duration %g seconds", (plan.t - start.t).seconds());
    RCLCPP_INFO(logger_, "segment 1 of %d", segments_.size());

    // Update status
    status.first_segment(orca_msgs::msg::Control::PLAN_LOCAL, segments_.size(),
      segments_[0]->to_str(), segments_[0]->type());
    RCLCPP_INFO(logger_, status.segment_info);

    // Set initial pose and twist
    status.pose = segments_[status.segment_idx]->plan();
    status.twist = segments_[status.segment_idx]->twist();
  }

  void LocalPlanner::add_keep_station_segment(FPStamped &plan, double seconds)
  {
    segments_.push_back(std::make_shared<Pause>(cxt_, plan, rclcpp::Duration::from_seconds(seconds)));
  }

  void LocalPlanner::add_vertical_segment(FPStamped &plan, double z)
  {
    segments_.push_back(TrapVelo::make_vertical(cxt_, plan, z));
  }

  void LocalPlanner::add_rotate_segment(FPStamped &plan, double yaw)
  {
    segments_.push_back(TrapVelo::make_rotate(cxt_, plan, yaw));
  }

  void LocalPlanner::add_line_segment(FPStamped &plan, double x, double y)
  {
    segments_.push_back(TrapVelo::make_line(cxt_, plan, x, y));
  }

  void LocalPlanner::add_pose_segment(FPStamped &plan, const FP &goal)
  {
    segments_.push_back(TrapVelo::make_pose(cxt_, plan, goal));
  }

  bool LocalPlanner::advance(const rclcpp::Duration &d, const FPStamped &estimate, orca::Efforts &efforts,
                             PlannerStatus &status)
  {
    // Update the plan
    if (segments_[status.segment_idx]->advance(d)) {
      // Continue in this segment
    } else if (++status.segment_idx < segments_.size()) {
      // Move to the next segment
      RCLCPP_INFO(logger_, "segment %d of %d", status.segment_idx + 1, segments_.size());

      // Update status
      status.next_segment(segments_[status.segment_idx]->to_str(), segments_[status.segment_idx]->type());
      RCLCPP_INFO(logger_, status.segment_info);
    } else {
      // Local plan is complete
      return false;
    }

    // Update pose and twist
    status.pose = segments_[status.segment_idx]->plan();
    status.twist = segments_[status.segment_idx]->twist();

    // Run PID controller and calculate efforts
    controller_->calc(d, status.pose.fp, estimate.fp, segments_[status.segment_idx]->ff(), efforts);

    return true;
  }

} // namespace orca_base
