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
  const BaseContext cxt_;                               // Parameters
  const fiducial_vlam_msgs::msg::Map map_;              // Map
  const PoseStamped start_;                             // Start point

  std::vector<std::shared_ptr<BaseMotion>> segments_;   // Trajectory segments
  int segment_idx_;                                     // Current segment
  nav_msgs::msg::Path planned_path_;                    // Path for rviz

  // Build a plan
  virtual void plan() = 0;

public:

  BaseMission(const rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
    const PoseStamped &start): logger_{logger}, cxt_{cxt}, map_{map}, start_{start} {}

  // Advance the controller, return true to continue
  bool advance(const double dt, const PoseStamped &curr, Acceleration &u_bar);

  const nav_msgs::msg::Path &planned_path() const { return planned_path_; }
};

//=====================================================================================
// KeepStationMission
// -- hover in place forever
//=====================================================================================

class KeepStationMission: public BaseMission
{
  void plan() override;

public:

  KeepStationMission(const rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
    const PoseStamped &start): BaseMission(logger, cxt, map, start) {}
};

//=====================================================================================
// DownRandomMission
// -- random zigzag mission for a down-facing camera
//=====================================================================================

class DownRandomMission: public BaseMission
{
  void plan() override;

public:

  DownRandomMission(const rclcpp::Logger &logger, const BaseContext &cxt, const fiducial_vlam_msgs::msg::Map &map,
    const PoseStamped &start): BaseMission(logger, cxt, map, start) {}
};

//=====================================================================================
// ForwardRandomMission
// -- random zigzag mission for a forward-facing camera
//=====================================================================================

// TODO


} // namespace orca_base

#endif //ORCA_BASE_MISSION_HPP
