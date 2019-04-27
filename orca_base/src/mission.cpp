#include "orca_base/mission.hpp"

#include <random>

namespace orca_base {

const rclcpp::Duration MIN_TIME{100000000}; // 0.1s

// TODO reconcile time (mission) vs. progress (motion)
// TODO combine line segments
// TODO build 3D odommap showing quality of odometry
// TODO build optimal path through odommap

//=====================================================================================
// Utilities
//=====================================================================================

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

//=====================================================================================
// Mission
//=====================================================================================

Mission::Mission(rclcpp::Logger logger, const orca_base::BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
  const PoseStamped &start):
  logger_{logger}
{
  // Waypoints are directly above markers
  std::vector<Pose> waypoints;
  for (auto i = map.poses.begin(); i != map.poses.end(); i++) {
    Pose waypoint;
    waypoint.from_msg(i->pose);
    waypoint.z = cxt.auv_z_target_;
    waypoints.push_back(waypoint);
  }

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
    segments_.push_back(std::make_shared<VerticalMotion>(logger, cxt, prev.pose, curr.pose));
  } else {
    RCLCPP_INFO(logger, "skip vertical");
  }
  prev = curr;

  // Travel to each marker
  for (auto i = waypoints.begin(); i != waypoints.end(); i++) {
    // Point in the direction fo travel
    curr.pose.yaw = atan2(i->y - curr.pose.y, i->x - curr.pose.x);
    if (set_time_yaw(cxt, prev, curr)) {
      path.push_back(curr);
      segments_.push_back(std::make_shared<RotateMotion>(logger, cxt, prev.pose, curr.pose));

    } else {
      RCLCPP_INFO(logger, "skip rotate");
    }
    prev = curr;

    // Run
    curr.pose.x = i->x;
    curr.pose.y = i->y;
    if (set_time_xy(cxt, prev, curr)) {
      path.push_back(curr);
      segments_.push_back(std::make_shared<LineMotion>(logger, cxt, prev.pose, curr.pose));

    } else {
      RCLCPP_INFO(logger, "skip line");
    }
    prev = curr;
  }

  // Generate path message
  planned_path_.header.stamp = start.t;
  planned_path_.header.frame_id = cxt.map_frame_;

  for (auto i = path.begin(); i != path.end(); i++) {
    geometry_msgs::msg::PoseStamped pose_msg;
    i->to_msg(pose_msg);
    planned_path_.poses.push_back(pose_msg);
  }

  // Start
  segment_idx_ = 0;
  RCLCPP_INFO(logger_, "mission has %d segments, segment 0", segments_.size());
}

bool Mission::advance(const double dt, const PoseStamped &curr, Acceleration &u_bar)
{
  if (segments_[segment_idx_]->advance(dt, curr.pose, u_bar)) {
    return true;
  }

  if (++segment_idx_ < segments_.size()) {
    RCLCPP_INFO(logger_, "mission segment %d", segment_idx_);
    return true;
  }

  RCLCPP_INFO(logger_, "mission complete");
  return false;
}

} // namespace orca_base
