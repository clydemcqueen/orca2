#ifndef ORCA_BASE_MAP_HPP
#define ORCA_BASE_MAP_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"

#include "orca_base/astar.hpp"
#include "orca_base/geometry.hpp"

namespace orca_base
{

  class Map
  {
    rclcpp::Logger logger_;

    // Marker map from vlam
    fiducial_vlam_msgs::msg::Map::SharedPtr vlam_map_;

    // A* solver
    std::shared_ptr<astar::Solver> solver_;

    // Return the index into map_.ids for a marker
    size_t index(int marker) const;

  public:

    explicit Map(const rclcpp::Logger &logger) : logger_{logger}
    {}

    // Initialize or update the map
    void set_vlam_map(fiducial_vlam_msgs::msg::Map::SharedPtr map);

    // Get the map
    fiducial_vlam_msgs::msg::Map::SharedPtr vlam_map() const
    { return vlam_map_; }

    // True if we have a good map
    bool ok()
    { return vlam_map_ != nullptr; }

    // Return the closest marker
    int closest_marker(const Pose &pose) const;

    // Use A* to generate a path from start_pose to destination_pose that stays close to the markers
    bool get_waypoints(const Pose &start_pose, const Pose &destination_pose, std::vector<Pose> &waypoints) const;
  };

} // namespace orca_base

#endif //ORCA_BASE_MAP_HPP
