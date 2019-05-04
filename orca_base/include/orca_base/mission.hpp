#ifndef ORCA_BASE_MISSION_HPP
#define ORCA_BASE_MISSION_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "nav_msgs/msg/path.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/motion.hpp"

namespace orca_base {

//=====================================================================================
// BaseMission
//=====================================================================================

class BaseMission
{
protected:

  rclcpp::Logger logger_;                               // ROS logger
  std::vector<std::shared_ptr<BaseMotion>> segments_;   // Trajectory segments
  int segment_idx_;                                     // Current segment
  nav_msgs::msg::Path planned_path_;                    // Path for rviz

public:

  BaseMission(const rclcpp::Logger &logger): logger_{logger} {}

  // Advance the controller, return true to continue
  bool advance(const double dt, const PoseStamped &curr, Acceleration &u_bar);

  const nav_msgs::msg::Path &planned_path() const { return planned_path_; }
};

//=====================================================================================
// ForwardKeepStationMission
// -- keep station facing a marker
//=====================================================================================

// TODO

//=====================================================================================
// ForwardRandomMission
// -- random zigzag mission for a forward-facing camera
//=====================================================================================

// TODO

//=====================================================================================
// DownRandomMission
// -- random zigzag mission for a down-facing camera
//=====================================================================================

class DownRandomMission: public BaseMission
{
  // Build a plan
  void plan(const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map, const PoseStamped &start);

public:

  DownRandomMission(const rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
    const PoseStamped &start);
};

} // namespace orca_base

#endif //ORCA_BASE_MISSION_HPP
