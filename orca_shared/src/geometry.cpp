#include "orca_shared/geometry.hpp"

namespace orca
{

  // TODO get from camera info
  const double hfov = 1.4;  // Horizontal field of view
  const double hres = 800;  // Horizontal resolution

  // TODO get from context
  const double marker_len = 0.1778;

  //=====================================================================================
  // Pose in world frame
  //=====================================================================================

  std::ostream &operator<<(std::ostream &os, Pose const &pose)
  {
    return os << "{" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.yaw << "}";
  }

  //=====================================================================================
  // PoseWithCovariance
  //=====================================================================================

  void PoseWithCovariance::from_msg(const geometry_msgs::msg::PoseWithCovariance &pose_with_covariance)
  {
    pose.from_msg(pose_with_covariance.pose);
    x_valid = (pose_with_covariance.covariance[0 * 7] < 1e4);
    y_valid = (pose_with_covariance.covariance[1 * 7] < 1e4);
    z_valid = (pose_with_covariance.covariance[2 * 7] < 1e4);
    yaw_valid = (pose_with_covariance.covariance[5 * 7] < 1e4);
  }

  //=====================================================================================
  // Acceleration in the world frame
  //=====================================================================================

  std::ostream &operator<<(std::ostream &os, Acceleration const &a)
  {
    return os << "{" << a.x << ", " << a.y << ", " << a.z << ", " << a.yaw << "}";
  }

  //=====================================================================================
  // Efforts
  //=====================================================================================

  std::ostream &operator<<(std::ostream &os, Efforts const &e)
  {
    return os << "{" << e.forward() << ", " << e.strafe() << ", " << e.vertical() << ", " << e.yaw() << "}";
  }

  //=====================================================================================
  // Observation -- observation of a marker from a camera
  //=====================================================================================

  void Observation::estimate_distance_and_yaw_from_corners()
  {
    // Assumptions:
    // -- camera is pointed forward
    // -- camera is mounted at base_link (not true, but OK for now)
    // -- pixels are square
    // -- markers are mounted vertically
    // -- roll and pitch are zero

    // Find the longest side of the marker in pixels
    double side01 = std::hypot(c0.x - c1.x, c0.y - c1.y);
    double side12 = std::hypot(c1.x - c2.x, c1.y - c2.y);
    double side23 = std::hypot(c2.x - c3.x, c2.y - c3.y);
    double side30 = std::hypot(c3.x - c0.x, c3.y - c0.y);
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
    double x = (c0.x + c1.x + c2.x + c3.x) / 4;

    yaw = hfov / 2 - x * hfov / hres;
  }

  void Observation::from_msg(const fiducial_vlam_msgs::msg::Observation &msg)
  {
    id = msg.id;
    c0.x = msg.x0;
    c1.x = msg.x1;
    c2.x = msg.x2;
    c3.x = msg.x3;
    c0.y = msg.y0;
    c1.y = msg.y1;
    c2.y = msg.y2;
    c3.y = msg.y3;
    estimate_distance_and_yaw_from_corners();
  }

  std::ostream &operator<<(std::ostream &os, Observation const &obs)
  {
    return os << "{marker: " << obs.id << ", distance: " << obs.distance << ", yaw: " << obs.yaw << "}";
  }

  //=====================================================================================
  // Fiducial pose (FP) -- pose and observations
  //=====================================================================================

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

  bool FP::good_obs(int id) const
  {
    for (auto i : observations) {
      if (i.id == id) {
        return true;
      }
    }

    return false;
  }

  bool FP::get_obs(int id, Observation &obs) const
  {
    for (auto i : observations) {
      if (i.id == id) {
        obs = i;
        return true;
      }
    }

    return false;
  }

  void FP::from_msgs(const fiducial_vlam_msgs::msg::Observations &obs,
                     const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg)
  {
    pose.from_msg(fcam_msg.pose);

    observations.clear();
    for (const auto &r : obs.observations) {
      Observation observation;
      observation.from_msg(r);
      observations.push_back(observation);
    }
  }

  //=====================================================================================
  // FPStamped -- pose and observations with timestamp
  //=====================================================================================

  void FPStamped::from_msgs(const fiducial_vlam_msgs::msg::Observations &obs,
                            const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg)
  {
    assert(obs.header.stamp == fcam_msg.header.stamp);

    t = obs.header.stamp;
    fp.from_msgs(obs, fcam_msg);
  }

  void FPStamped::add_to_path(nav_msgs::msg::Path &path) const
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = path.header.frame_id;
    fp.pose.pose.to_msg(msg.pose);
    path.poses.push_back(msg);
  }

} // namespace orca_shared