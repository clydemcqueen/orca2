#ifndef ORCA_BASE_MAP_HPP
#define ORCA_BASE_MAP_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"

#include <utility>

#include "orca_base/geometry.hpp"

namespace orca_base
{

  class Map
  {
    rclcpp::Logger logger_;
    fiducial_vlam_msgs::msg::Map map_;

    // Return the index into map_.ids for a marker
    size_t index(int marker) const;

  public:

    Map(const rclcpp::Logger &logger, fiducial_vlam_msgs::msg::Map map) :
      logger_{logger},
      map_{std::move(map)}
    {}

    // These functions:
    // -- generally ignore z, and work better with down-facing cameras
    // -- don't consider visibility, e.g., closest_visible_marker might be more useful

    // Returns marker closest to a pose
    int closest_marker(const Pose &pose) const;

    // Returns a waypoint from start_marker to dest_marker, or dest_marker if there are no good waypoints
    int next_waypoint(int start_marker, int dest_marker) const;

    // Given a start pose, find a good pose to keep station at a marker
    Pose keep_station_pose(int marker, const Pose &start_pose) const;

    // TODO add waypoint enum functions from mission.cpp
  };

} // namespace orca_base

#endif //ORCA_BASE_MAP_HPP
