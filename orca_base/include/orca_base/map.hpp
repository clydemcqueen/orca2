#ifndef ORCA_BASE_MAP_HPP
#define ORCA_BASE_MAP_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "rclcpp/logger.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"

#include "orca_shared/geometry.hpp"

#include "orca_base/astar.hpp"
#include "orca_base/auv_context.hpp"
#include "orca_base/fp.hpp"

namespace orca_base
{
  struct Marker
  {
    double marker_length{0};
    int id{NOT_A_MARKER};
    geometry_msgs::msg::Pose marker_f_map;
    tf2::Vector3 corner0_f_map;
    tf2::Vector3 corner1_f_map;
    tf2::Vector3 corner2_f_map;
    tf2::Vector3 corner3_f_map;

    Marker() = default;

    Marker(int id, const geometry_msgs::msg::Pose &_marker_f_map, double marker_length);

    bool predict_observation(const AUVContext &cxt, const image_geometry::PinholeCameraModel &cam_model,
                             const tf2::Transform &t_cam_map, Observation &obs) const;
  };

  class Map
  {
    rclcpp::Logger logger_;
    const AUVContext &cxt_;

    // Marker map from vlam
    fiducial_vlam_msgs::msg::Map::SharedPtr vlam_map_;

    // Markers in ROS world coordinates
    std::vector<Marker> markers_;

  public:

    explicit Map(const rclcpp::Logger &logger, const AUVContext &cxt) : logger_{logger}, cxt_{cxt}
    {}

    // Initialize or update the map
    void set_vlam_map(fiducial_vlam_msgs::msg::Map::SharedPtr map);

    // Get marker length
    double marker_length() const
    { return vlam_map_->marker_length; }

    // Get markers
    std::vector<Marker> markers() const
    { return markers_; }

    // True if we have a good map
    bool ok()
    { return vlam_map_ != nullptr; }

    // Use A* to generate a path from start_pose to destination_pose that stays close to the markers
    bool get_waypoints(const orca::Pose &start_pose, const orca::Pose &destination_pose,
                       std::vector<orca::Pose> &waypoints) const;

    // Find a marker by id, return true if found
    bool find_marker(int marker_id, Marker &result) const;
  };

} // namespace orca_base

#endif //ORCA_BASE_MAP_HPP
