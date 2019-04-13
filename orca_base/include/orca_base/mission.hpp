#ifndef ORCA_BASE_MISSION_HPP
#define ORCA_BASE_MISSION_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "nav_msgs/msg/path.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/motion.hpp"

namespace orca_base {

class Mission
{
  rclcpp::Logger logger_;                               // ROS logger
  std::vector<std::shared_ptr<BaseMotion>> segments_;   // Trajectory segments
  int segment_idx_;                                     // Current segment
  nav_msgs::msg::Path planned_path_;                    // Path for rviz

public:

  Mission(rclcpp::Logger logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
    const PoseStamped &start);

  // Advance the controller, return true to continue
  bool advance(const double dt, const PoseStamped &curr, Acceleration &u_bar);

  const nav_msgs::msg::Path &planned_path() const { return planned_path_; }
};

} // namespace orca_base

#endif //ORCA_BASE_MISSION_HPP