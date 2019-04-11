#include "orca_base/planner.hpp"

namespace orca_base {

const double CRUISING_Z = -0.7;
const double SPEED = 0.5;           // m/s

double distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
{
  return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
}

nav_msgs::msg::Path plan(const rclcpp::Time &msg_time, const fiducial_vlam_msgs::msg::Map &map, const OrcaPose &start)
{
  nav_msgs::msg::Path path{};
  path.header.stamp = msg_time;
  path.header.frame_id = "map";

  rclcpp::Time running_time = msg_time + STABILIZE;

  // Start pose
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.header.stamp = running_time;
  start.to_msg(p.pose);
  path.poses.push_back(p);

  // Travel to each marker in order
  for (int i = 0; i < map.poses.size(); i++) {

    // Compute time to travel to the next waypoint
    rcl_duration_value_t travel_time = distance(p.pose.position, map.poses[i].pose.position) / SPEED * 1e9;
    running_time = running_time + rclcpp::Duration{travel_time} + STABILIZE;

    // Add waypoint to path
    p.header.stamp = running_time;
    p.pose = map.poses[i].pose;
    p.pose.position.z = CRUISING_Z;
    path.poses.push_back(p);
  }

  return path;

  // TODO always point in the direction of travel
  // TODO start at closest marker
  // TODO verify that auv will never lose odom
}

} // namespace orca_base
