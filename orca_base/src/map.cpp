#include "orca_base/map.hpp"

namespace orca_base
{

  // Max dead reckoning distance in meters
  constexpr double MAX_DEAD_RECKONING_DISTANCE = 6;

  // Pseudo marker IDs
  constexpr astar::node_type START_ID = -1;
  constexpr astar::node_type DESTINATION_ID = -2;

  bool Map::get_waypoints(const Pose &start_pose, const Pose &destination_pose, std::vector<Pose> &waypoints) const
  {
    waypoints.clear();

    // Create a map of marker ids to poses, including the start and destination pose
    std::map<astar::node_type, Pose> poses;

    // Add start pose to map
    auto start_z_target = start_pose;
    start_z_target.z = cxt_.auv_z_target_;
    poses[START_ID] = start_z_target;

    // Add destination pose to map
    auto destination_z_target = destination_pose;
    destination_z_target.z = cxt_.auv_z_target_;
    poses[DESTINATION_ID] = destination_z_target;

    // Add all of the markers to the map
    for (size_t i = 0; i < vlam_map_->ids.size(); ++i) {
      Pose pose;
      pose.from_msg(vlam_map_->poses[i].pose);
      pose.z = cxt_.auv_z_target_;
      poses[vlam_map_->ids[i]] = pose;
    }

    // Enumerate all edges between markers that are < MAX_DEAD_RECKONING_DISTANCE
    std::vector<astar::Edge> short_paths;
    for (auto i = poses.begin(); i != poses.end(); ++i) {
      for (auto j = std::next(i); j != poses.end(); ++j) {
        auto distance = i->second.distance_xy(j->second);
        if (distance < MAX_DEAD_RECKONING_DISTANCE) {
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

} // namespace orca_base
