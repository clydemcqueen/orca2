#include "orca_base/planner.hpp"

#include <random>

namespace orca_base
{

  //=====================================================================================
  // Utilities
  //=====================================================================================

  geometry_msgs::msg::Pose map_to_world(const geometry_msgs::msg::Pose &marker_f_map)
  {
    // Rotation from vlam map frame to ROS world frame
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

  void PlannerBase::add_keep_station_segment(Pose &plan, double seconds)
  {
    segments_.push_back(std::make_shared<Pause>(logger_, cxt_, plan, seconds));
    controllers_.push_back(std::make_shared<SimpleController>(cxt_));
  }

  void PlannerBase::add_vertical_segment(Pose &plan, double z)
  {
    Pose goal = plan;
    goal.z = z;
    if (plan.distance_z(goal) > EPSILON_PLAN_XYZ) {
      segments_.push_back(std::make_shared<VerticalSegment>(logger_, cxt_, plan, goal));
      controllers_.push_back(std::make_shared<SimpleController>(cxt_));
    } else {
      RCLCPP_INFO(logger_, "skip vertical");
    }
    plan = goal;
  }

  void PlannerBase::add_rotate_segment(Pose &plan, double yaw)
  {
    Pose goal = plan;
    goal.yaw = yaw;
    if (plan.distance_yaw(goal) > EPSILON_PLAN_YAW) {
      segments_.push_back(std::make_shared<RotateSegment>(logger_, cxt_, plan, goal));
      controllers_.push_back(std::make_shared<SimpleController>(cxt_));
    } else {
      RCLCPP_INFO(logger_, "skip rotate");
    }
    plan = goal;
  }

  void PlannerBase::add_line_segment(Pose &plan, double x, double y)
  {
    Pose goal = plan;
    goal.x = x;
    goal.y = y;
    if (plan.distance_xy(goal) > EPSILON_PLAN_XYZ) {
//      if (segments_.empty() || !segments_.back()->extend(plan, goal)) {
      segments_.push_back(std::make_shared<LineSegment>(logger_, cxt_, plan, goal));
      controllers_.push_back(std::make_shared<SimpleController>(cxt_));
//      }
    } else {
      RCLCPP_INFO(logger_, "skip line");
    }
    plan = goal;
  }

  void PlannerBase::plan_trajectory(const PoseStamped &start)
  {
    RCLCPP_INFO(logger_, "plan trajectory to (%g, %g, %g), %g",
                targets_[target_idx_].x, targets_[target_idx_].y, targets_[target_idx_].z, targets_[target_idx_].yaw);

    // Generate a series of waypoints to minimize dead reckoning TODO
    std::vector<Pose> waypoints;
    waypoints.push_back(targets_[target_idx_]);

    // Plan trajectory through the waypoints, possibly keeping station if this is the last target
    plan_trajectory(waypoints, start, target_idx_ == targets_.size() - 1 && keep_station_);
  }

  void PlannerBase::plan_trajectory(const std::vector<Pose> &waypoints, const PoseStamped &start, bool keep_station)
  {
    RCLCPP_INFO(logger_, "plan trajectory through %d waypoint(s)", waypoints.size() - 1);

    // Clear existing segments
    segments_.clear();
    controllers_.clear();
    segment_idx_ = 0;

    // Start pose
    Pose plan = start.pose;

    // Travel to each waypoint, breaking down z, yaw and xy phases
    for (auto &waypoint : waypoints) {
      // Ascend/descend to target z
      add_vertical_segment(plan, waypoint.z);

      if (plan.distance_xy(waypoint.x, waypoint.y) > EPSILON_PLAN_XYZ) {
        // Point in the direction of travel
        add_rotate_segment(plan, atan2(waypoint.y - plan.y, waypoint.x - plan.x));

        // Settle at the current rotation
        if (cxt_.auv_open_water_) {
          add_keep_station_segment(plan, 5);
        }

        // Travel
        add_line_segment(plan, waypoint.x, waypoint.y);

        // Settle at the new marker
        if (cxt_.auv_open_water_) {
          add_keep_station_segment(plan, 5);
        }
      } else {
        RCLCPP_DEBUG(logger_, "skip travel");
      }
    }

    // Rotate to the final yaw
    add_rotate_segment(plan, waypoints.back().yaw);

    if (keep_station) {
      // Keep station at the last waypoint
      segments_.push_back(std::make_shared<Pause>(logger_, cxt_, plan, 1e6));
      controllers_.push_back(std::make_shared<SimpleController>(cxt_));
    }

    // Create a path for diagnostics
    if (!segments_.empty()) {
      planned_path_.header.stamp = start.t;
      planned_path_.header.frame_id = cxt_.map_frame_;

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = start.t;

      for (auto &i : segments_) {
        planned_path_.header.frame_id = cxt_.map_frame_;
        i->plan().to_msg(pose_msg.pose);
        planned_path_.poses.push_back(pose_msg);
      }

      // Add last goal pose
      segments_.back()->goal().to_msg(pose_msg.pose);
      planned_path_.poses.push_back(pose_msg);
    }

    assert(!segments_.empty());
  }

  int PlannerBase::advance(double dt, Pose &plan, const nav_msgs::msg::Odometry &estimate, Acceleration &u_bar,
                           const std::function<void(double completed, double total)> &send_feedback)
  {
    PoseStamped current_pose;
    current_pose.from_msg(estimate);

    if (segments_.empty()) {
      if (full_pose(estimate)) {
        // Generate a trajectory to the first target
        RCLCPP_INFO(logger_, "bootstrap plan");
        plan_trajectory(current_pose);
      } else {
        // Can't bootstrap
        RCLCPP_ERROR(logger_, "unknown pose, can't bootstrap");
        return AdvanceRC::FAILURE;
      }
    }

    Acceleration ff;

    if (segments_[segment_idx_]->advance(dt)) {

      // Advance the current motion segment
      plan = segments_[segment_idx_]->plan();
      ff = segments_[segment_idx_]->ff();

    } else if (++segment_idx_ < segments_.size()) {

      // The segment is done, move to the next segment
      RCLCPP_INFO(logger_, "segment %d of %d", segment_idx_ + 1, segments_.size());
      plan = segments_[segment_idx_]->plan();
      ff = segments_[segment_idx_]->ff();

    } else if (++target_idx_ < targets_.size()) {

      // Current trajectory complete, move to the next target
      RCLCPP_INFO(logger_, "target %d of %d", target_idx_ + 1, targets_.size());
      send_feedback(target_idx_, targets_.size());

      if (full_pose(estimate)) {
        // Start from known location
        plan_trajectory(current_pose);
        RCLCPP_INFO(logger_, "planning for next target from known pose");
      } else {
        // Still dead reckoning
        // Future: run through recovery actions to find the target marker
        RCLCPP_WARN(logger_, "didn't find target, planning for next target anyway");
        PoseStamped plan_stamped;
        plan_stamped.pose = targets_[target_idx_ - 1];
        plan_stamped.t = estimate.header.stamp;
        plan_trajectory(plan_stamped);
      }

      plan = segments_[segment_idx_]->plan();
      ff = segments_[segment_idx_]->ff();

    } else {
      return AdvanceRC::SUCCESS;
    }

    // Compute acceleration
    controllers_[segment_idx_]->calc(cxt_, dt, plan, estimate, ff, u_bar);

    // If error is > 0.5m, then replan
    if (full_pose(estimate) && current_pose.pose.distance_xy(plan) > 0.8) {
      RCLCPP_INFO(logger_, "off by %g meters, replan to existing target", current_pose.pose.distance_xy(plan));
      plan_trajectory(current_pose);
    }

    return AdvanceRC::CONTINUE;
  }

  //=====================================================================================
  // DownSequencePlanner
  //=====================================================================================

  DownSequencePlanner::DownSequencePlanner(const rclcpp::Logger &logger, const BaseContext &cxt,
                                           fiducial_vlam_msgs::msg::Map map, bool random) :
    PlannerBase{logger, cxt, std::move(map), true}
  {
    // Targets are directly above the markers
    for (const auto &pose : map_.poses) {
      Pose target;
      target.from_msg(pose.pose);
      target.z = cxt_.auv_z_target_;
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
                                                 fiducial_vlam_msgs::msg::Map map, bool random) :
    PlannerBase{logger, cxt, std::move(map), false}
  {
    // Targets are directly in front of the markers
    for (const auto &pose : map_.poses) {
      geometry_msgs::msg::Pose marker_f_world = map_to_world(pose.pose);
      Pose target;
      target.from_msg(marker_f_world);
      target.x += cos(target.yaw) * cxt_.auv_xy_distance_;
      target.y += sin(target.yaw) * cxt_.auv_xy_distance_;
      target.z = cxt_.auv_z_target_;
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
