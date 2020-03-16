#include "orca_base/fp.hpp"

#include <iostream>
#include <iomanip>

namespace orca_base
{

  //=====================================================================================
  // Observation -- observation of a marker from a camera
  //=====================================================================================

  Observation::Observation(const fiducial_vlam_msgs::msg::Observation &msg,
                           double marker_length, double hfov, double hres)
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
    estimate_distance_and_bearing(marker_length, hfov, hres);
  }

  Observation::Observation(int _id, const cv::Point2d &_c0, const cv::Point2d &_c1, const cv::Point2d &_c2,
                           const cv::Point2d &_c3, double marker_length, double hfov, double hres)
  {
    id = _id;
    c0 = _c0;
    c1 = _c1;
    c2 = _c2;
    c3 = _c3;
    estimate_distance_and_bearing(marker_length, hfov, hres);
  }

  void Observation::estimate_distance_and_bearing(double marker_length, double hfov, double hres)
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

    distance = marker_length / sin(longest_side / hres * hfov);

    double center_x = (c0.x + c1.x + c2.x + c3.x) / 4;

    bearing = hfov / 2 - center_x * hfov / hres;
  }

  void Observation::estimate_corners(double marker_length, double hfov, double hres, double vres)
  {
    // Same assumptions as above, plus:
    // -- marker is facing the camera
    // -- camera and marker are at the same height

    assert(distance > 0);

    double longest_side = hres / hfov * asin(marker_length / distance);

    double center_x = hres * (0.5 - bearing / hfov);
    double center_y = vres / 2;

    c0.x = center_x - longest_side / 2;
    c1.x = center_x + longest_side / 2;
    c2.x = center_x + longest_side / 2;
    c3.x = center_x - longest_side / 2;

    c0.y = center_y - longest_side / 2;
    c1.y = center_y - longest_side / 2;
    c2.y = center_y + longest_side / 2;
    c3.y = center_y + longest_side / 2;
  }

  orca_msgs::msg::Observation Observation::to_msg() const
  {
    orca_msgs::msg::Observation result;
    result.vlam.id = id;
    result.vlam.x0 = c0.x;
    result.vlam.y0 = c0.y;
    result.vlam.x1 = c1.x;
    result.vlam.y1 = c1.y;
    result.vlam.x2 = c2.x;
    result.vlam.y2 = c2.y;
    result.vlam.x3 = c3.x;
    result.vlam.y3 = c3.y;
    result.distance = distance;
    result.bearing = bearing;
    return result;
  }

  std::ostream &operator<<(std::ostream &os, Observation const &obs)
  {
    return os << std::fixed << std::setprecision(2)
              << "{marker: " << obs.id << ", distance: " << obs.distance << ", bearing: " << obs.bearing << "}";
  }

  //=====================================================================================
  // ObservationStamped -- observation with a timestamp
  //=====================================================================================

  std::ostream &operator<<(std::ostream &os, ObservationStamped const &obs)
  {
    return os << obs.o;
  }

  //=====================================================================================
  // Fiducial pose (FP) -- pose and observations
  //=====================================================================================

  bool FP::good_pose(double good_pose_dist) const
  {
    return pose.good_pose() && closest_observation() < good_pose_dist;
  }

  bool FP::has_good_observation(double good_obs_dist) const
  {
    return closest_observation() < good_obs_dist;
  }

  bool FP::get_observation(int id, Observation &obs) const
  {
    for (const auto &i : observations) {
      if (i.id == id) {
        obs = i;
        return true;
      }
    }

    return false;
  }

  bool FP::get_good_observation(double good_obs_dist, int id, Observation &obs) const
  {
    return get_observation(id, obs) && obs.distance < good_obs_dist;
  }

  double FP::closest_observation() const
  {
    Observation obs;
    return get_closest_observation(obs);
  }

  double FP::get_closest_observation(Observation &obs) const
  {
    double closest = std::numeric_limits<double>::max();

    for (const auto &i : observations) {
      if (i.distance < closest) {
        closest = i.distance;
        obs = i;
      }
    }

    return closest;
  }

  void FP::from_msgs(const fiducial_vlam_msgs::msg::Observations &obs,
                     const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg,
                     double marker_length, double hfov, double hres)
  {
    pose.from_msg(fcam_msg.pose);

    observations.clear();
    for (const auto &r : obs.observations) {
      observations.emplace_back(r, marker_length, hfov, hres);
    }
  }

  void FP::set_good_z(double z)
  {
    // Override the z value
    pose.pose.z = z;
    pose.z_valid = true;
  }

  std::ostream &operator<<(std::ostream &os, FP const &fp)
  {
    os << fp.pose.pose;
  }

  //=====================================================================================
  // FPStamped -- pose and observations with timestamp
  //=====================================================================================

  bool FPStamped::get_good_observation(double good_obs_dist, int id, ObservationStamped &obs) const
  {
    obs.t = t;
    return fp.get_good_observation(good_obs_dist, id, obs.o);
  }

  double FPStamped::get_closest_observation(ObservationStamped &obs) const
  {
    obs.t = t;
    return fp.get_closest_observation(obs.o);
  }

  void FPStamped::from_msgs(const fiducial_vlam_msgs::msg::Observations &obs,
                            const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg,
                            double marker_length, double hfov, double hres)
  {
    assert(obs.header.stamp == fcam_msg.header.stamp);

    t = obs.header.stamp;
    fp.from_msgs(obs, fcam_msg, marker_length, hfov, hres);
  }

  void FPStamped::add_to_path(nav_msgs::msg::Path &path) const
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = path.header.frame_id;
    msg.pose = fp.pose.pose.to_msg();
    path.poses.push_back(msg);
  }

  std::ostream &operator<<(std::ostream &os, FPStamped const &fp)
  {
    os << fp.fp.pose.pose;
  }

}