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
  // BasePlanner
  //=====================================================================================

  void BasePlanner::add_vertical_segment(Pose &plan, double z)
  {
    Pose goal = plan;
    goal.z = z;
    if (plan.distance_z(goal) > EPSILON_PLAN_XYZ) {
      segments_.push_back(std::make_shared<VerticalSegment>(logger_, cxt_, plan, goal));
    } else {
      RCLCPP_DEBUG(logger_, "skip vertical");
    }
    plan = goal;
  }

  void BasePlanner::add_rotate_segment(Pose &plan, double yaw)
  {
    Pose goal = plan;
    goal.yaw = yaw;
    if (plan.distance_yaw(goal) > EPSILON_PLAN_YAW) {
      segments_.push_back(std::make_shared<RotateSegment>(logger_, cxt_, plan, goal));
    } else {
      RCLCPP_DEBUG(logger_, "skip rotate");
    }
    plan = goal;
  }

  void BasePlanner::add_line_segment(Pose &plan, double x, double y)
  {
    Pose goal = plan;
    goal.x = x;
    goal.y = y;
    if (plan.distance_xy(goal) > EPSILON_PLAN_XYZ) {
      if (!segments_.back()->extend(plan, goal)) {
        segments_.push_back(std::make_shared<LineSegment>(logger_, cxt_, plan, goal));
      }
    } else {
      RCLCPP_DEBUG(logger_, "skip line");
    }
    plan = goal;
  }

  void BasePlanner::plan_target(const Pose &target, const PoseStamped &start, bool keep_station)
  {
    std::vector<Pose> waypoints;
    waypoints.push_back(target);
    plan_waypoints(waypoints, start, keep_station);
  }

  void BasePlanner::plan_waypoints(const std::vector<Pose> &waypoints, const PoseStamped &start, bool keep_station)
  {
    RCLCPP_INFO(logger_, "plan trajectory through %d waypoint(s), keeping station at (%g, %g, %g), %g",
                waypoints.size() - 1, waypoints.back().x, waypoints.back().y, waypoints.back().z, waypoints.back().yaw);

    // Start pose
    Pose plan = start.pose;

    // Travel to each waypoint, breaking down z, yaw and xy phases
    for (auto &waypoint : waypoints) {
      // Ascend/descend to target z
      add_vertical_segment(plan, waypoint.z);

      if (plan.distance_xy(waypoint.x, waypoint.y) > EPSILON_PLAN_XYZ) {
        // Point in the direction of travel
        add_rotate_segment(plan, atan2(waypoint.y - plan.y, waypoint.x - plan.x));

        // Travel
        add_line_segment(plan, waypoint.x, waypoint.y);
      } else {
        RCLCPP_DEBUG(logger_, "skip travel");
      }
    }

    // Rotate to the final yaw
    add_rotate_segment(plan, waypoints.back().yaw);

    if (keep_station) {
      // Keep station at the last waypoint
      segments_.push_back(std::make_shared<BaseSegment>(logger_, cxt_, plan, plan));
    }

    // Create a path for diagnostics
    planned_path_.header.stamp = start.t;
    planned_path_.header.frame_id = cxt_.map_frame_;
    for (auto &i : segments_) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = start.t;
      planned_path_.header.frame_id = cxt_.map_frame_;
      i->plan().to_msg(pose_msg.pose);
      planned_path_.poses.push_back(pose_msg);
    }
  }

  //=====================================================================================
  // KeepStationPlanner
  //=====================================================================================

  void KeepStationPlanner::plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    plan_target(start.pose, start, true);
  }

  //=====================================================================================
  // KeepOriginPlanner
  //=====================================================================================

  void KeepOriginPlanner::plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    Pose target;
    target.z = cxt_.auv_z_target_;
    plan_target(target, start, false); // TODO
  }

  //=====================================================================================
  // DownSequencePlanner
  //=====================================================================================

  void DownSequencePlanner::plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    bool up_down = false; // !random_;

    // Waypoints are directly above markers
    std::vector<Pose> waypoints;
    for (const auto &pose : map.poses) {
      Pose waypoint;
      waypoint.from_msg(pose.pose);
      waypoint.z = cxt_.auv_z_target_;
      waypoints.push_back(waypoint);

      if (up_down) {
        waypoint.z = -3;
        waypoints.push_back(waypoint);
      }
    }

    if (random_) {
      // Shuffle waypoints
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(waypoints.begin(), waypoints.end(), g);
    }

    plan_waypoints(waypoints, start, false);
  }

  //=====================================================================================
  // ForwardSequencePlanner
  //=====================================================================================

  void ForwardSequencePlanner::plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // Waypoints are directly in front of markers
    std::vector<Pose> waypoints;
    for (const auto &pose : map.poses) {
      geometry_msgs::msg::Pose marker_f_world = map_to_world(pose.pose);
      Pose waypoint;
      waypoint.from_msg(marker_f_world);
      waypoint.x += cos(waypoint.yaw) * cxt_.auv_xy_distance_;
      waypoint.y += sin(waypoint.yaw) * cxt_.auv_xy_distance_;
      waypoint.z = cxt_.auv_z_target_;
      waypoints.push_back(waypoint);
    }

    if (random_) {
      // Shuffle waypoints
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(waypoints.begin(), waypoints.end(), g);
    }

    plan_waypoints(waypoints, start, false);
  }

  //=====================================================================================
  // Body planners
  //=====================================================================================

  // Move forward and back
  void BodyXPlanner::plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // TODO
  }

  // Move left and right
  void BodyYPlanner::plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // TODO
  }

  // Move up and down
  void BodyZPlanner::plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // TODO
  }

  // Move ccw and cw
  void BodyYawPlanner::plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // TODO
  }

} // namespace orca_base
