#include "orca_base/fp.hpp"

#include <iostream>

namespace orca_base
{

  //=====================================================================================
  // Observation -- observation of a marker from a camera
  //=====================================================================================

  void Observation::estimate_distance_and_yaw_from_corners(double marker_length, double hfov, double hres)
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

    yaw = hfov / 2 - center_x * hfov / hres;
  }

  void Observation::estimate_corners_from_distance_and_yaw(double marker_length, double hfov, double hres, double vres)
  {
    // Same assumptions as above, plus:
    // -- marker is facing the camera
    // -- camera and marker are at the same height

    assert(distance > 0);

    double longest_side = hres / hfov * asin(marker_length / distance);

    double center_x = hres * (0.5 - yaw / hfov);
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

  void Observation::from_msg(const fiducial_vlam_msgs::msg::Observation &msg,
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
    estimate_distance_and_yaw_from_corners(marker_length, hfov, hres);
  }

  std::ostream &operator<<(std::ostream &os, Observation const &obs)
  {
    return os << "{marker: " << obs.id << ", distance: " << obs.distance << ", yaw: " << obs.yaw << "}";
  }

  //=====================================================================================
  // ObservationStamped -- observation with a timestamp
  //=====================================================================================

  void ObservationStamped::from_msg(const rclcpp::Time &stamp, const fiducial_vlam_msgs::msg::Observation &msg,
                                    double marker_length, double hfov, double hres)
  {
    t = stamp;
    o.from_msg(msg, marker_length, hfov, hres);
  }

  std::ostream &operator<<(std::ostream &os, ObservationStamped const &obs)
  {
    return os << obs.o;
  }

  //=====================================================================================
  // Fiducial pose (FP) -- pose and observations
  //=====================================================================================

  double FP::closest_obs(Observation &obs) const
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

  double FP::closest_obs() const
  {
    Observation obs;
    return closest_obs(obs);
  }

  bool FP::good_obs(int id) const
  {
    for (const auto &i : observations) {
      if (i.id == id) {
        return true;
      }
    }

    return false;
  }

  bool FP::get_obs(int id, Observation &obs) const
  {
    for (const auto &i : observations) {
      if (i.id == id) {
        obs = i;
        return true;
      }
    }

    return false;
  }

  void FP::from_msgs(const fiducial_vlam_msgs::msg::Observations &obs,
                     const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg,
                     double marker_length, double hfov, double hres)
  {
    pose.from_msg(fcam_msg.pose);

    observations.clear();
    for (const auto &r : obs.observations) {
      Observation observation;
      observation.from_msg(r, marker_length, hfov, hres);
      observations.push_back(observation);
    }
  }

  std::ostream &operator<<(std::ostream &os, FP const &fp)
  {
    os << fp.pose.pose;
  }

  //=====================================================================================
  // FPStamped -- pose and observations with timestamp
  //=====================================================================================

  double FPStamped::closest_obs(ObservationStamped &obs) const
  {
    obs.t = t;
    return fp.closest_obs(obs.o);
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