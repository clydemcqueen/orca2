#ifndef ORCA_BASE_PLANNER_HPP
#define ORCA_BASE_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "fiducial_vlam_msgs/msg/map.hpp"
#include "nav_msgs/msg/path.hpp"

#include "orca_base/geometry.hpp"

namespace orca_base {

const rclcpp::Duration STABILIZE{5000000000};

nav_msgs::msg::Path plan(const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start);

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_HPP
