#include "orca_base/map.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_base
{

  // Pseudo marker IDs
  constexpr astar::node_type START_ID = -1;
  constexpr astar::node_type DESTINATION_ID = -2;

  //=====================================================================================
  // Utilities
  //=====================================================================================

  // Rotate markers from vlam map frame (derived from OpenCV) to ROS world frame (typically called "map")
  // Also see orca_gazebo/build_worlds.py
  geometry_msgs::msg::Pose vlam_to_map(const geometry_msgs::msg::Pose &marker_f_vlam)
  {
    const static tf2::Quaternion t_map_vlam(0, 0, -sqrt(0.5), sqrt(0.5));

    tf2::Quaternion t_vlam_marker(marker_f_vlam.orientation.x, marker_f_vlam.orientation.y,
                                  marker_f_vlam.orientation.z, marker_f_vlam.orientation.w);
    tf2::Quaternion t_map_marker = t_map_vlam * t_vlam_marker;

    geometry_msgs::msg::Pose marker_f_map = marker_f_vlam;
    marker_f_map.orientation.x = t_map_marker.x();
    marker_f_map.orientation.y = t_map_marker.y();
    marker_f_map.orientation.z = t_map_marker.z();
    marker_f_map.orientation.w = t_map_marker.w();
    return marker_f_map;
  }

  inline cv::Point3d tf_to_cv(tf2::Vector3 p)
  {
    return cv::Point3d(p.x(), p.y(), p.z());
  }

  // Return true if this point is inside of the rectangle [[0, 0], [2*cx, 2*cy]]
  inline bool in_frame(const cv::Point2d &p, double cx, double cy)
  {
    return p.x >= 0 && p.x <= 2 * cx && p.y >= 0 && p.y <= 2 * cy;
  }

  //=====================================================================================
  // Marker
  //=====================================================================================

  Marker::Marker(int _id, const geometry_msgs::msg::Pose &_marker_f_map, double _marker_length) :
    marker_length{_marker_length}, id{_id}, marker_f_map{_marker_f_map}
  {
    // Build corners in marker frame
    tf2::Vector3 corner0_f_marker(-marker_length / 2.f, marker_length / 2.f, 0.f);
    tf2::Vector3 corner1_f_marker(marker_length / 2.f, marker_length / 2.f, 0.f);
    tf2::Vector3 corner2_f_marker(marker_length / 2.f, -marker_length / 2.f, 0.f);
    tf2::Vector3 corner3_f_marker(-marker_length / 2.f, -marker_length / 2.f, 0.f);

    // Get transform from marker frame to map frame
    tf2::Transform t_map_marker;
    tf2::fromMsg(marker_f_map, t_map_marker);

    // Transform corners into map frame
    corner0_f_map = t_map_marker * corner0_f_marker;
    corner1_f_map = t_map_marker * corner1_f_marker;
    corner2_f_map = t_map_marker * corner2_f_marker;
    corner3_f_map = t_map_marker * corner3_f_marker;
  }

  bool Marker::predict_observation(const AUVContext &cxt, const image_geometry::PinholeCameraModel &cam_model,
                                   const tf2::Transform &t_cam_map, Observation &obs) const
  {
    // Camera frame: x right, y down, z forward

    // Transform corners from map frame to camera frame
    auto corner0_f_cam = t_cam_map * corner0_f_map;
    auto corner1_f_cam = t_cam_map * corner1_f_map;
    auto corner2_f_cam = t_cam_map * corner2_f_map;
    auto corner3_f_cam = t_cam_map * corner3_f_map;

    // Ignore markers that are behind the camera
    if (corner0_f_cam.z() < 0 || corner1_f_cam.z() < 0 || corner2_f_cam.z() < 0 || corner3_f_cam.z() < 0) {
      return false;
    }

    // Ignore markers that are not facing the camera TODO

    // Project corners onto the image plane
    auto c0 = cam_model.project3dToPixel(tf_to_cv(corner0_f_cam));
    auto c1 = cam_model.project3dToPixel(tf_to_cv(corner1_f_cam));
    auto c2 = cam_model.project3dToPixel(tf_to_cv(corner2_f_cam));
    auto c3 = cam_model.project3dToPixel(tf_to_cv(corner3_f_cam));

    // Ignore markers that are outside of the visible frame
    if (!in_frame(c0, cam_model.cx(), cam_model.cy()) ||
        !in_frame(c1, cam_model.cx(), cam_model.cy()) ||
        !in_frame(c2, cam_model.cx(), cam_model.cy()) ||
        !in_frame(c3, cam_model.cx(), cam_model.cy())) {
      return false;
    }

    obs = Observation(id, c0, c1, c2, c3, marker_length, cxt.fcam_hfov_, cxt.fcam_hres_);

    return true;
  }

  //=====================================================================================
  // Map
  //=====================================================================================

  void Map::set_vlam_map(fiducial_vlam_msgs::msg::Map::SharedPtr map)
  {
    // Save map
    vlam_map_ = std::move(map);

    // Build markers_
    markers_.clear();
    for (size_t i = 0; i < vlam_map_->ids.size(); ++i) {
      markers_.emplace_back(vlam_map_->ids[i], vlam_map_->poses[i].pose, vlam_map_->marker_length);
    }
  }

  // TODO ignores marker orientation, so only works for down-facing cameras and markers on the seafloor
  bool Map::get_waypoints(const orca::Pose &start_pose, const orca::Pose &destination_pose,
                          std::vector<orca::Pose> &waypoints) const
  {
    waypoints.clear();

    // Create a map of marker ids to poses, including the start and destination pose
    std::map<astar::node_type, orca::Pose> poses;

    // Add start pose to map
    auto start_z_target = start_pose;
    start_z_target.z = cxt_.planner_z_target_;
    poses[START_ID] = start_z_target;

    // Add destination pose to map
    auto destination_z_target = destination_pose;
    destination_z_target.z = cxt_.planner_z_target_;
    poses[DESTINATION_ID] = destination_z_target;

    // Add all of the markers to the map
    for (size_t i = 0; i < vlam_map_->ids.size(); ++i) {
      orca::Pose pose;
      pose.from_msg(vlam_map_->poses[i].pose);
      pose.z = cxt_.planner_z_target_;
      poses[vlam_map_->ids[i]] = pose;
    }

    // Enumerate all edges between markers that are < MAX_DEAD_RECKONING_DISTANCE
    std::vector<astar::Edge> short_paths;
    for (auto i = poses.begin(); i != poses.end(); ++i) {
      for (auto j = std::next(i); j != poses.end(); ++j) {
        auto distance = i->second.distance_xy(j->second);
        if (distance < cxt_.planner_max_dead_reckon_dist_) {
          short_paths.emplace_back(i->first, j->first, distance);
        }
      }
    }

    // Create an A* solver that we can use to navigate between markers that are further away
    auto solver = std::make_shared<astar::Solver>(short_paths, [&](astar::node_type a, astar::node_type b) -> double
    {
      return poses[a].distance_xy(poses[b]);
    });

    // Find the shortest path from the start pose to the destination pose through markers
    std::vector<astar::node_type> path;
    if (solver->find_shortest_path(START_ID, DESTINATION_ID, path)) {
      for (auto marker : path) {
        waypoints.push_back(poses[marker]);
      }
      return true;
    } else {
      RCLCPP_ERROR(logger_, "A* failed to find a path through the markers");
      return false;
    }
  }

  bool Map::find_marker(int marker_id, Marker &result) const
  {
    for (const auto &marker : markers_) {
      if (marker.id == marker_id) {
        result = marker;
        return true;
      }
    }

    return false;
  }

} // namespace orca_base
