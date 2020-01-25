#include "orca_base/planner.hpp"

#include <random>

using namespace orca;

namespace orca_base
{

  constexpr double MAX_POSE_ERROR = 0.6;

  //=====================================================================================
  // Utilities
  //=====================================================================================

  // Rotation from vlam map frame to ROS world frame
  geometry_msgs::msg::Pose map_to_world(const geometry_msgs::msg::Pose &marker_f_map)
  {
    const static tf2::Quaternion t_world_map(0, 0, -sqrt(0.5), sqrt(0.5));

    tf2::Quaternion t_map_marker(marker_f_map.orientation.x, marker_f_map.orientation.y, marker_f_map.orientation.z,
                                 marker_f_map.orientation.w);
    tf2::Quaternion t_world_marker = t_world_map * t_map_marker;

    geometry_msgs::msg::Pose marker_f_world = marker_f_map;
    marker_f_world.orientation.x = t_world_marker.x();
    marker_f_world.orientation.y = t_world_marker.y();
    marker_f_world.orientation.z = t_world_marker.z();
    marker_f_world.orientation.w = t_world_marker.w();
    return marker_f_world;
  }

  //=====================================================================================
  // PlannerBase
  //=====================================================================================

  void PlannerBase::add_keep_station_segment(FP &plan, double seconds)
  {
    segments_.push_back(std::make_shared<Pause>(logger_, cxt_, plan, seconds));
    controllers_.push_back(std::make_shared<SimpleController>(cxt_));
  }

  void PlannerBase::add_vertical_segment(FP &plan, double z)
  {
    FP goal = plan;
    goal.pose.pose.z = z;
    if (plan.pose.pose.distance_z(goal.pose.pose) > EPSILON_PLAN_XYZ) {
      segments_.push_back(std::make_shared<VerticalSegment>(logger_, cxt_, plan, goal));
      controllers_.push_back(std::make_shared<SimpleController>(cxt_));
    } else {
      RCLCPP_INFO(logger_, "skip vertical");
    }
    plan = goal;
  }

  void PlannerBase::add_rotate_segment(FP &plan, double yaw)
  {
    FP goal = plan;
    goal.pose.pose.yaw = yaw;
    if (plan.pose.pose.distance_yaw(goal.pose.pose) > EPSILON_PLAN_YAW) {
      segments_.push_back(std::make_shared<RotateSegment>(logger_, cxt_, plan, goal));
      controllers_.push_back(std::make_shared<SimpleController>(cxt_));
    } else {
      RCLCPP_INFO(logger_, "skip rotate");
    }
    plan = goal;
  }

  void PlannerBase::add_line_segment(FP &plan, double x, double y)
  {
    FP goal = plan;
    goal.pose.pose.x = x;
    goal.pose.pose.y = y;
    if (plan.pose.pose.distance_xy(goal.pose.pose) > EPSILON_PLAN_XYZ) {
      segments_.push_back(std::make_shared<LineSegment>(logger_, cxt_, plan, goal));
      controllers_.push_back(std::make_shared<SimpleController>(cxt_));
    } else {
      RCLCPP_INFO(logger_, "skip line");
    }
    plan = goal;
  }

  void PlannerBase::plan_trajectory(const FP &start)
  {
    RCLCPP_INFO(logger_, "plan trajectory to (%g, %g, %g), %g",
                targets_[target_idx_].pose.pose.x, targets_[target_idx_].pose.pose.y, targets_[target_idx_].pose.pose.z,
                targets_[target_idx_].pose.pose.yaw);

    // Generate a series of waypoints to minimize dead reckoning
    std::vector<Pose> waypoints;
    if (!map_.get_waypoints(start.pose.pose, targets_[target_idx_].pose.pose, waypoints)) {
      RCLCPP_ERROR(logger_, "feeling lucky");
      waypoints.push_back(targets_[target_idx_].pose.pose);
    }

    // Plan trajectory through the waypoints
    plan_trajectory(waypoints, start);
  }

  void PlannerBase::plan_trajectory(const std::vector<Pose> &waypoints, const FP &start)
  {
    RCLCPP_INFO(logger_, "plan trajectory through %d waypoint(s):", waypoints.size() - 1);
    for (auto waypoint : waypoints) {
      RCLCPP_INFO_STREAM(logger_, waypoint);
    }

    // Clear existing segments
    segments_.clear();
    controllers_.clear();
    segment_idx_ = 0;

    // Start pose
    FP plan = start;

    // Travel to each waypoint, breaking down z, yaw and xy phases
    for (auto &waypoint : waypoints) {
      // Ascend/descend to target z
      add_vertical_segment(plan, waypoint.z);

      if (plan.pose.pose.distance_xy(waypoint.x, waypoint.y) > EPSILON_PLAN_XYZ) {
        // Point in the direction of travel
        add_rotate_segment(plan, atan2(waypoint.y - plan.pose.pose.y, waypoint.x - plan.pose.pose.x));

        // Travel
        add_line_segment(plan, waypoint.x, waypoint.y);
      } else {
        RCLCPP_DEBUG(logger_, "skip travel");
      }
    }

    // Always rotate to the target yaw
    add_rotate_segment(plan, targets_[target_idx_].pose.pose.yaw);

    // Keep station at the last target
    if (keep_station_ && target_idx_ == targets_.size() - 1) {
      add_keep_station_segment(plan, 1e6);
    }

    // Create a path for diagnostics
    if (!segments_.empty()) {
      planned_path_.header.frame_id = cxt_.map_frame_;
      planned_path_.poses.clear();

      geometry_msgs::msg::PoseStamped pose_msg;
      for (auto &i : segments_) {
        planned_path_.header.frame_id = cxt_.map_frame_;
        i->plan().pose.pose.to_msg(pose_msg.pose);
        planned_path_.poses.push_back(pose_msg);
      }

      // Add last goal pose
      segments_.back()->goal().pose.pose.to_msg(pose_msg.pose);
      planned_path_.poses.push_back(pose_msg);
    }

    assert(!segments_.empty());
    RCLCPP_INFO(logger_, "segment 1 of %d", segments_.size());
    segments_[0]->log_info();
  }

  int PlannerBase::advance(double dt, FP &plan, const FP &estimate, Acceleration &u_bar,
                           const std::function<void(double completed, double total)> &send_feedback)
  {
    if (segments_.empty()) {
      if (estimate.pose.full_pose()) {
        // Generate a trajectory to the first target
        RCLCPP_INFO(logger_, "bootstrap plan");
        plan_trajectory(estimate);
      } else {
        RCLCPP_ERROR(logger_, "unknown pose, can't bootstrap");
        return AdvanceRC::FAILURE;
      }
    }

    if (segments_[segment_idx_]->advance(dt)) {

      // Continue in this segment

    } else if (++segment_idx_ < segments_.size()) {

      // Move to the next segment
      RCLCPP_INFO(logger_, "segment %d of %d", segment_idx_ + 1, segments_.size());
      segments_[segment_idx_]->log_info();

    } else if (++target_idx_ < targets_.size()) {

      // Move to the next target
      RCLCPP_INFO(logger_, "target %d of %d", target_idx_ + 1, targets_.size());
      send_feedback(target_idx_, targets_.size());

      if (estimate.pose.full_pose()) {
        // Start from known location
        plan_trajectory(estimate);
        RCLCPP_INFO(logger_, "planning for next target from known pose");
      } else {
        // Plan a trajectory as if the AUV is at the previous target (it probably isn't)
        // Future: run through recovery actions to find a good pose
        RCLCPP_WARN(logger_, "didn't find target, planning for next target anyway");
        plan_trajectory(targets_[target_idx_ - 1]);
      }

    } else {
      return AdvanceRC::SUCCESS;
    }

    plan = segments_[segment_idx_]->plan();
    Acceleration ff = segments_[segment_idx_]->ff();

    // Compute acceleration
    controllers_[segment_idx_]->calc(cxt_, dt, plan, estimate, ff, u_bar);

    // If error is > MAX_POSE_ERROR, then replan
    if (estimate.pose.full_pose() && estimate.pose.pose.distance_xy(plan.pose.pose) > MAX_POSE_ERROR) {
      RCLCPP_WARN(logger_, "off by %g meters, replan to existing target", estimate.pose.pose.distance_xy(plan.pose.pose));
      plan_trajectory(estimate);
    }

    return AdvanceRC::CONTINUE;
  }

  //=====================================================================================
  // DownSequencePlanner
  //=====================================================================================

  DownSequencePlanner::DownSequencePlanner(const rclcpp::Logger &logger, const BaseContext &cxt,
                                           Map map, bool random) :
    PlannerBase{logger, cxt, std::move(map), true}
  {
    // Targets are directly above the markers
    for (const auto &pose : map_.vlam_map()->poses) {
      FP target;
      target.pose.pose.from_msg(pose.pose);
      target.pose.pose.z = cxt_.auv_z_target_;
      targets_.push_back(target);
    }

    if (random) {
      // Shuffle targets
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(targets_.begin(), targets_.end(), g);
    }
  }

  //=====================================================================================
  // ForwardSequencePlanner
  //=====================================================================================

  ForwardSequencePlanner::ForwardSequencePlanner(const rclcpp::Logger &logger, const BaseContext &cxt,
                                                 Map map, bool random) :
    PlannerBase{logger, cxt, std::move(map), false}
  {
    // Targets are directly in front of the markers
    for (const auto &pose : map_.vlam_map()->poses) {
      geometry_msgs::msg::Pose marker_f_world = map_to_world(pose.pose);
      FP target;
      target.pose.pose.from_msg(marker_f_world);
      target.pose.pose.x += cos(target.pose.pose.yaw) * cxt_.auv_xy_distance_;
      target.pose.pose.y += sin(target.pose.pose.yaw) * cxt_.auv_xy_distance_;
      target.pose.pose.z = cxt_.auv_z_target_;
      targets_.push_back(target);
    }

    if (random) {
      // Shuffle targets
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(targets_.begin(), targets_.end(), g);
    }
  }

} // namespace orca_base
