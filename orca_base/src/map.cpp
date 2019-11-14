#include "orca_base/map.hpp"

namespace orca_base
{

  constexpr double MAX_DEAD_RECKONING_DISTANCE = 7;

  double distance_xy(const geometry_msgs::msg::PoseWithCovariance &start,
                     const geometry_msgs::msg::PoseWithCovariance &dest)
  {
    return std::hypot(dest.pose.position.x - start.pose.position.x, dest.pose.position.y - start.pose.position.y);
  }

  size_t Map::index(int marker) const
  {
    for (size_t i = 0; i < vlam_map_->ids.size(); ++i) {
      if (marker == vlam_map_->ids[i]) {
        return i;
      }
    }

    throw (std::runtime_error("marker not found"));
  }

  void Map::set_vlam_map(fiducial_vlam_msgs::msg::Map::SharedPtr map)
  {
    vlam_map_ = std::move(map);

    // Enumerate all paths between markers that are < MAX_DEAD_RECKONING_DISTANCE
    std::vector<astar::Edge> short_paths;
    for (size_t i = 0; i < vlam_map_->ids.size(); ++i) {
      for (size_t j = i + 1; j < vlam_map_->ids.size(); ++j) {
        double distance = distance_xy(vlam_map_->poses[i], vlam_map_->poses[j]);
        if (distance < MAX_DEAD_RECKONING_DISTANCE) {
          short_paths.emplace_back(vlam_map_->ids[i], vlam_map_->ids[j], distance);
        }
      }
    }

    // Create an A* solver that we can use to navigate between markers that are further away
    solver_ = std::make_shared<astar::Solver>(short_paths, [this](astar::node_type a, astar::node_type b) -> double
    {
      return distance_xy(vlam_map_->poses[index(a)], vlam_map_->poses[index(b)]);
    });
  }

  int Map::closest_marker(const Pose &pose) const
  {
    int closest_marker = -1;
    double closest_marker_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < vlam_map_->ids.size(); ++i) {
      auto distance = pose.distance_xy(vlam_map_->poses[i].pose);
      if (distance < closest_marker_distance) {
        closest_marker = vlam_map_->ids[i];
        closest_marker_distance = distance;
      }
    }

    RCLCPP_INFO(logger_, "marker %d is closest to (%g, %g)", closest_marker, pose.x, pose.y);
    return closest_marker;
  }

  bool Map::get_waypoints(const Pose &start_pose, const Pose &destination_pose, std::vector<Pose> &waypoints) const
  {
    waypoints.clear();

    // Find the shortest path from the start pose to the destination pose through markers
    std::vector<astar::node_type> path;
    if (solver_->find_shortest_path(closest_marker(start_pose), closest_marker(destination_pose), path)) {
      for (auto marker : path) {
        Pose pose;
        pose.from_msg(vlam_map_->poses[index(marker)].pose);
        waypoints.push_back(pose);
      }
      waypoints.push_back(destination_pose);
      return true;
    } else {
      RCLCPP_ERROR(logger_, "A* failed to find a path through the markers");
      return false;
    }
  }

} // namespace orca_base
