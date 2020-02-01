#ifndef ORCA_BASE_MAP_HPP
#define ORCA_BASE_MAP_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "image_geometry/pinhole_camera_model.h"

#include "orca_shared/geometry.hpp"

#include "orca_base/astar.hpp"
#include "orca_base/base_context.hpp"

namespace orca_base
{
  struct Marker
  {
    int id{orca::NOT_A_MARKER};
    geometry_msgs::msg::Pose marker_f_map;
    tf2::Vector3 corner0_f_map;
    tf2::Vector3 corner1_f_map;
    tf2::Vector3 corner2_f_map;
    tf2::Vector3 corner3_f_map;

    Marker(int id, const geometry_msgs::msg::Pose &_marker_f_map);

    bool predict_observation(const image_geometry::PinholeCameraModel &cam_model, const tf2::Transform &t_cam_map,
                             orca::Observation &obs) const;
  };

  class Map
  {
    rclcpp::Logger logger_;
    const BaseContext &cxt_;

    // Marker map from vlam
    fiducial_vlam_msgs::msg::Map::SharedPtr vlam_map_;

    // Markers in ROS world coordinates
    std::vector<Marker> markers_;

  public:

    explicit Map(const rclcpp::Logger &logger, const BaseContext &cxt) : logger_{logger}, cxt_{cxt}
    {}

    // Initialize or update the map
    void set_vlam_map(fiducial_vlam_msgs::msg::Map::SharedPtr map);

    // Get markers
    std::vector<Marker> markers() const
    { return markers_; }

    // True if we have a good map
    bool ok()
    { return vlam_map_ != nullptr; }

    // Use A* to generate a path from start_pose to destination_pose that stays close to the markers
    bool get_waypoints(const orca::Pose &start_pose, const orca::Pose &destination_pose,
                       std::vector<orca::Pose> &waypoints) const;
  };

} // namespace orca_base

#endif //ORCA_BASE_MAP_HPP
