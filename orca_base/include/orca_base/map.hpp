#ifndef ORCA_BASE_MAP_HPP
#define ORCA_BASE_MAP_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"

#include "orca_shared/geometry.hpp"

#include "orca_base/astar.hpp"
#include "orca_base/base_context.hpp"

namespace orca_base
{

  class Map
  {
    rclcpp::Logger logger_;
    const BaseContext &cxt_;

    // Marker map from vlam
    fiducial_vlam_msgs::msg::Map::SharedPtr vlam_map_;

  public:

    explicit Map(const rclcpp::Logger &logger, const BaseContext &cxt) : logger_{logger}, cxt_{cxt}
    {}

    // Initialize or update the map
    void set_vlam_map(fiducial_vlam_msgs::msg::Map::SharedPtr map)
    { vlam_map_ = std::move(map); }

    // Get the map
    fiducial_vlam_msgs::msg::Map::SharedPtr vlam_map() const
    { return vlam_map_; }

    // True if we have a good map
    bool ok()
    { return vlam_map_ != nullptr; }

    // Use A* to generate a path from start_pose to destination_pose that stays close to the markers
    bool get_waypoints(const orca::Pose &start_pose, const orca::Pose &destination_pose, std::vector<orca::Pose> &waypoints) const;
  };

} // namespace orca_base

#endif //ORCA_BASE_MAP_HPP
