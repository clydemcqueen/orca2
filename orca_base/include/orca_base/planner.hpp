#ifndef ORCA_BASE_PLANNER_HPP
#define ORCA_BASE_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "fiducial_vlam_msgs/msg/map.hpp"
#include "nav_msgs/msg/path.hpp"

#include "model.hpp"

namespace orca_base {

const rclcpp::Duration STABILIZE{2000000000}; // TODO move to shared hpp

// TODO move msg_time into OrcaPose
nav_msgs::msg::Path plan(const rclcpp::Time &msg_time, const fiducial_vlam_msgs::msg::Map &map, const OrcaPose &start);

} // namespace orca_base

#endif //ORCA_BASE_PLANNER_HPP
