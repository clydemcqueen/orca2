#include "orca_base/planner.hpp"

#include <random>

namespace orca_base
{
  
  //=====================================================================================
  // Utilities
  //=====================================================================================

  const rclcpp::Duration MIN_TIME{100000000}; // 0.1s

  bool set_time_xy(const orca_base::BaseContext &cxt, const PoseStamped &p1, PoseStamped &p2)
  {
    rclcpp::Duration d(sqrt(pow(p2.pose.x - p1.pose.x, 2) + pow(p2.pose.y - p1.pose.y, 2)) / cxt.auv_xy_speed_ * 1e9);
    p2.t = p1.t + d;
    return d > MIN_TIME;
  }

  bool set_time_z(const orca_base::BaseContext &cxt, const PoseStamped &p1, PoseStamped &p2)
  {
    rclcpp::Duration d(std::abs(p2.pose.z - p1.pose.z) / cxt.auv_z_speed_ * 1e9);
    p2.t = p1.t + d;
    return d > MIN_TIME;
  }

  bool set_time_yaw(const orca_base::BaseContext &cxt, const PoseStamped &p1, PoseStamped &p2)
  {
    rclcpp::Duration d(std::abs(norm_angle(p2.pose.yaw - p1.pose.yaw)) / cxt.auv_yaw_speed_ * 1e9);
    p2.t = p1.t + d;
    return d > MIN_TIME;
  }

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
  // KeepStationPlanner
  //=====================================================================================

  void KeepStationPlanner::plan(rclcpp::Logger &logger, const BaseContext &cxt,
                                const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // Keep station over the start pose
    segments_.push_back(std::make_shared<BaseSegment>(logger, cxt, start.pose, start.pose));

    // Trivial path message
    geometry_msgs::msg::PoseStamped pose_msg;
    start.to_msg(pose_msg);
    planned_path_.header.stamp = start.t;
    planned_path_.header.frame_id = cxt.map_frame_;
    planned_path_.poses.push_back(pose_msg);
  }

  //=====================================================================================
  // OriginPlanner
  //=====================================================================================

  void OriginPlanner::plan(rclcpp::Logger &logger, const BaseContext &cxt,
                           const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // Keep station below the origin
    PoseStamped target;
    target.t = start.t;
    target.pose.z = cxt.auv_z_target_;
    segments_.push_back(std::make_shared<BaseSegment>(logger, cxt, target.pose, target.pose));

    // Trivial path message
    geometry_msgs::msg::PoseStamped pose_msg;
    target.to_msg(pose_msg);
    planned_path_.header.stamp = start.t;
    planned_path_.header.frame_id = cxt.map_frame_;
    planned_path_.poses.push_back(pose_msg);
  }

  //=====================================================================================
  // RandomPlanner
  //
  // Generate a path between waypoints.
  //=====================================================================================

  void RandomPlanner::plan_from_waypoints(rclcpp::Logger &logger, const BaseContext &cxt, std::vector<Pose> &waypoints,
                                          const PoseStamped &start)
  {
    // Shuffle waypoints
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(waypoints.begin(), waypoints.end(), g);

    // Path may contain multiple poses per waypoint, as we break down Z, YAW and XY phases
    std::vector<PoseStamped> path;

    // Start pose
    PoseStamped prev = start;
    path.push_back(prev);

    // Move to cruising z
    PoseStamped curr = start;
    curr.pose.z = cxt.auv_z_target_;
    if (set_time_z(cxt, prev, curr)) {
      path.push_back(curr);
      segments_.push_back(std::make_shared<VerticalSegment>(logger, cxt, prev.pose, curr.pose));
    } else {
      RCLCPP_INFO(logger, "skip vertical");
    }
    prev = curr;

    // Travel to each waypoint
    for (auto &waypoint : waypoints) {
      // Point in the direction of travel
      curr.pose.yaw = atan2(waypoint.y - curr.pose.y, waypoint.x - curr.pose.x);
      if (set_time_yaw(cxt, prev, curr)) {
        path.push_back(curr);
        segments_.push_back(std::make_shared<RotateSegment>(logger, cxt, prev.pose, curr.pose));

      } else {
        RCLCPP_INFO(logger, "skip rotate");
      }
      prev = curr;

      // Run
      curr.pose.x = waypoint.x;
      curr.pose.y = waypoint.y;
      if (set_time_xy(cxt, prev, curr)) {
        path.push_back(curr);
        segments_.push_back(std::make_shared<LineSegment>(logger, cxt, prev.pose, curr.pose));

      } else {
        RCLCPP_INFO(logger, "skip line");
      }
      prev = curr;
    }

    // Generate path message
    planned_path_.header.stamp = start.t;
    planned_path_.header.frame_id = cxt.map_frame_;

    for (auto &i : path) {
      geometry_msgs::msg::PoseStamped pose_msg;
      i.to_msg(pose_msg);
      planned_path_.poses.push_back(pose_msg);
    }
  }

  //=====================================================================================
  // DownRandomPlanner
  //=====================================================================================

  void DownRandomPlanner::plan(rclcpp::Logger &logger, const BaseContext &cxt,
                               const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // Waypoints are directly above markers
    std::vector<Pose> waypoints;
    for (const auto &pose : map.poses) {
      Pose waypoint;
      waypoint.from_msg(pose.pose);
      waypoint.z = cxt.auv_z_target_;
      waypoints.push_back(waypoint);
    }

    plan_from_waypoints(logger, cxt, waypoints, start);
  }

  //=====================================================================================
  // ForwardRandomPlanner
  //=====================================================================================

  void ForwardRandomPlanner::plan(rclcpp::Logger &logger, const BaseContext &cxt,
                                  const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // Waypoints are directly in front of markers
    std::vector<Pose> waypoints;
    for (const auto &pose : map.poses) {
      geometry_msgs::msg::Pose marker_f_world = map_to_world(pose.pose);
      Pose waypoint;
      waypoint.from_msg(marker_f_world);
      waypoint.x += cos(waypoint.yaw) * cxt.auv_xy_distance_;
      waypoint.y += sin(waypoint.yaw) * cxt.auv_xy_distance_;
      waypoint.z = cxt.auv_z_target_;
      waypoints.push_back(waypoint);
    }

    plan_from_waypoints(logger, cxt, waypoints, start);
  }

  //=====================================================================================
  // Body planners
  //=====================================================================================

  // Move forward and back
  void BodyXPlanner::plan(rclcpp::Logger &logger, const BaseContext &cxt,
                          const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // First waypoint is at target z
    PoseStamped wp1 = start;
    wp1.pose.z = cxt.auv_z_target_;

    // Second waypoint is a few m ahead
    PoseStamped wp2 = wp1;
    wp2.pose.x += cos(wp2.pose.yaw) * cxt.auv_xy_distance_;
    wp2.pose.y += sin(wp2.pose.yaw) * cxt.auv_xy_distance_;

    // Trajectory segments
    segments_.push_back(std::make_shared<VerticalSegment>(logger, cxt, start.pose, wp1.pose));
    segments_.push_back(std::make_shared<LineSegment>(logger, cxt, wp1.pose, wp2.pose));
    segments_.push_back(std::make_shared<LineSegment>(logger, cxt, wp2.pose, wp1.pose));

    // Planned path
    planned_path_.header.stamp = start.t;
    planned_path_.header.frame_id = cxt.map_frame_;
    geometry_msgs::msg::PoseStamped pose_msg;
    start.to_msg(pose_msg);
    planned_path_.poses.push_back(pose_msg);
    wp1.to_msg(pose_msg);
    planned_path_.poses.push_back(pose_msg);
    wp2.to_msg(pose_msg);
    planned_path_.poses.push_back(pose_msg);
  }

  // Move left and right
  void BodyYPlanner::plan(rclcpp::Logger &logger, const BaseContext &cxt,
                          const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // TODO
  }

  // Move up and down
  void BodyZPlanner::plan(rclcpp::Logger &logger, const BaseContext &cxt,
                          const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // TODO
  }

  // Move ccw and cw
  void BodyYawPlanner::plan(rclcpp::Logger &logger, const BaseContext &cxt,
                            const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start)
  {
    // TODO
  }

} // namespace orca_base
