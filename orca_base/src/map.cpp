#include "orca_base/map.hpp"

namespace orca_base
{

  size_t Map::index(int marker) const
  {
    for (size_t i = 0; i < map_.ids.size(); ++i) {
      if (marker == map_.ids[i]) {
        return i;
      }
    }

    throw (std::runtime_error("marker not found"));
  }

  int Map::closest_marker(const Pose &pose) const
  {
    int closest_marker = -1;
    double closest_marker_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < map_.ids.size(); ++i) {
      auto distance = pose.distance_xy(map_.poses[i].pose);
      if (distance < closest_marker_distance) {
        closest_marker = map_.ids[i];
        closest_marker_distance = distance;
      }
    }

    return closest_marker;
  }

  double distance_xy(const geometry_msgs::msg::PoseWithCovariance &start,
                     const geometry_msgs::msg::PoseWithCovariance &dest)
  {
    return std::hypot(dest.pose.position.x - start.pose.position.x, dest.pose.position.y - start.pose.position.y);
  }

  int Map::next_waypoint(int start_marker, int dest_marker) const
  {
    // TODO
    return -1;
  }

  Pose Map::keep_station_pose(int marker, const Pose &start_pose) const
  {
    // TODO
    return {};
  }
} // namespace orca_base
