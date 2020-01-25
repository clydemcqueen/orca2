#include "orca_shared/geometry.hpp"

// TODO move all code from geometry.hpp to here

namespace orca
{

  void PoseWithCovariance::from_msg(const geometry_msgs::msg::PoseWithCovariance &pose_with_covariance)
  {
    pose.from_msg(pose_with_covariance.pose);
    x_valid = pose_with_covariance.covariance[0 * 7] < 1e4;
    y_valid = pose_with_covariance.covariance[1 * 7] < 1e4;
    z_valid = pose_with_covariance.covariance[2 * 7] < 1e4;
    yaw_valid = pose_with_covariance.covariance[5 * 7] < 1e4;
  }

  void Observation::from_msg(const fiducial_vlam_msgs::msg::Observation &msg)
  {
    id = msg.id;

    // Assumptions:
    // -- camera is pointed forward
    // -- camera is mounted at base_link (not true, but OK for now)
    // -- pixels are square
    // -- markers are mounted vertically
    // -- roll and pitch are zero

    // TODO get from context or observations.camera_info
    double hfov = 1.4;            // Horizontal field of view
    double hres = 800;            // Horizontal resolution
    double marker_len = 0.1778;   // Marker length

    // Find the longest side of the marker in pixels
    double side01 = std::hypot(msg.x0 - msg.x1, msg.y0 - msg.y1);
    double side12 = std::hypot(msg.x1 - msg.x2, msg.y1 - msg.y2);
    double side23 = std::hypot(msg.x2 - msg.x3, msg.y2 - msg.y3);
    double side30 = std::hypot(msg.x3 - msg.x0, msg.y3 - msg.y0);
    double longest_side = side01;
    if (side12 > longest_side) {
      longest_side = side12;
    }
    if (side23 > longest_side) {
      longest_side = side23;
    }
    if (side30 > longest_side) {
      longest_side = side30;
    }

    distance = marker_len / sin(longest_side / hres * hfov);

    // Center of marker
    double x = (msg.x0 + msg.x1 + msg.x2 + msg.x3) / 4;

    yaw = hfov / 2 - x * hfov / hres;
  }

  double FP::closest_obs() const
  {
    double closest = std::numeric_limits<double>::max();

    for (auto i : observations) {
      if (i.distance < closest) {
        closest = i.distance;
      }
    }

    return closest;
  }

  void FP::from_msgs(const fiducial_vlam_msgs::msg::Observations &obs, const nav_msgs::msg::Odometry &odom)
  {
    pose.from_msg(odom.pose);

    observations.clear();
    for (const auto &r : obs.observations) {
      Observation observation;
      observation.from_msg(r);
      observations.push_back(observation);
    }
  }

  void FPStamped::from_msgs(const fiducial_vlam_msgs::msg::Observations &obs, const nav_msgs::msg::Odometry &odom)
  {
    assert(obs.header.stamp == odom.header.stamp);

    t = obs.header.stamp;
    fp.from_msgs(obs, odom);
  }

  void FPStamped::add_to_path(nav_msgs::msg::Path &path) const
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = path.header.frame_id;
    fp.pose.pose.to_msg(msg.pose);
    path.poses.push_back(msg);
  }

  std::ostream &operator<<(std::ostream &os, Pose const &pose)
  {
    return os << "{" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.yaw << "}";
  }

  std::ostream &operator<<(std::ostream &os, Observation const &obs)
  {
    return os << "{marker: " << obs.id << ", distance: " << obs.distance << ", yaw: " << obs.yaw << "}";
  }

} // namespace orca_shared