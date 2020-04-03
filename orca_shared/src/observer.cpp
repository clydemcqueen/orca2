#include "orca_shared/observer.hpp"

#include <iostream>
#include <iomanip>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// WORK-IN-PROGRESS

namespace orca
{

  //=====================================================================================
  // Utilities
  //=====================================================================================

  tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose &pose)
  {
    tf2::Transform result;
    tf2::fromMsg(pose, result);
    return result;
  }

  geometry_msgs::msg::Pose transform_to_pose(const tf2::Transform &transform)
  {
    geometry_msgs::msg::Pose result;
    tf2::toMsg(transform, result);
    return result;
  }

  //=====================================================================================
  // Polar
  //=====================================================================================

  std::ostream &operator<<(std::ostream &os, Polar const &o)
  {
    return os << std::fixed << std::setprecision(2)
              << "{id: " << o.id << ", distance: " << o.distance << ", bearing: " << o.bearing << "}";
  }

  //=====================================================================================
  // PolarStamped
  //=====================================================================================

  std::ostream &operator<<(std::ostream &os, PolarStamped const &o)
  {
    return os << o.p;
  }

  //=====================================================================================
  // Polars
  //=====================================================================================

  Polar Polars::get(int id) const
  {
    for (const auto &i : v) {
      if (i.id == id) {
        return i;
      }
    }
    return {};
  }

  Polar Polars::closest() const
  {
    Polar result = {};
    for (const auto &i : v) {
      if (i.distance < result.distance) {
        result = i;
      }
    }
    return result;
  }

  std::ostream &operator<<(std::ostream &os, Polars const &o)
  {
    for (const auto &i : o.v) {
      os << i;
    }
    return os;
  }

  //=====================================================================================
  // Observer
  //=====================================================================================

  Observer::Observer(const orca_msgs::msg::FiducialPose &msg)
  {
    // TODO
//    marker_length_ = msg.marker_length;
//    cam_model_.fromCameraInfo(msg.camera_info);
//    t_base_cam_ = pose_to_transform(msg.cam_f_base);
    t_cam_base_ = t_base_cam_.inverse();
  }

  /**
   * Estimate distance and bearing from the corners
   *
   * Assume that the ray from the camera to the marker is orthogonal to at least one side of the marker.
   * This is likely if:
   * -- the camera is facing forward
   * -- the markers are mounted vertically
   * -- roll and pitch are zero
   *
   * If this assumption fails the distance will be too high.
   */
  Polar Observer::get_polar(const Observation &observation)
  {
    // Use the camera model to project the 2d corner points to 3d rays
    // The z value is always 1.0
    auto ray0 = cam_model_.projectPixelTo3dRay(observation.c0);
    auto ray1 = cam_model_.projectPixelTo3dRay(observation.c1);
    auto ray2 = cam_model_.projectPixelTo3dRay(observation.c2);
    auto ray3 = cam_model_.projectPixelTo3dRay(observation.c3);

    // Find the longest side
    auto side01 = std::hypot(ray0.x - ray1.x, ray0.y - ray1.y);
    auto side12 = std::hypot(ray1.x - ray2.x, ray1.y - ray2.y);
    auto side23 = std::hypot(ray2.x - ray3.x, ray2.y - ray3.y);
    auto side30 = std::hypot(ray3.x - ray0.x, ray3.y - ray0.y);
    auto longest_side = side01;
    if (side12 > longest_side) {
      longest_side = side12;
    }
    if (side23 > longest_side) {
      longest_side = side23;
    }
    if (side30 > longest_side) {
      longest_side = side30;
    }

    // Estimate the scale factor
    auto scale_factor = marker_length_ / longest_side;

    // Find the center of the marker
    tf2::Vector3 center_f_cam{(ray0.x + ray1.x + ray2.x + ray3.x) * scale_factor / 4,
                              (ray0.y + ray1.y + ray2.y + ray3.y) * scale_factor / 4,
                              scale_factor};

    // Transform the center point from the camera frame to the base frame
    auto center_f_base = t_base_cam_ * center_f_cam;

    // Calculate distance and bearing
    return {observation.id,
            std::hypot(center_f_base.x(), center_f_base.y()),
            std::atan2(center_f_base.y(), center_f_base.x())};
  }

  /**
   * Estimate the corners from distance and bearing
   *
   * Same assumption as above, plus:
   * -- the marker is facing the camera
   * -- the camera and the marker are at the same height
   */
  Observation Observer::get_observation(const Polar &polar)
  {
    // Find the center point
    tf2::Vector3 center_f_base{std::cos(polar.bearing) * polar.distance, std::sin(polar.bearing) * polar.distance, 0};

    // Transform the center point from the base frame to the camera frame
    auto center_f_cam = t_cam_base_ * center_f_base;

    // Build corners in the camera frame
    cv::Point3d c0_f_cam{center_f_cam.x() - marker_length_ / 2, center_f_cam.y() - marker_length_ / 2,
                         center_f_cam.z()};
    cv::Point3d c1_f_cam{center_f_cam.x() + marker_length_ / 2, center_f_cam.y() - marker_length_ / 2,
                         center_f_cam.z()};
    cv::Point3d c2_f_cam{center_f_cam.x() + marker_length_ / 2, center_f_cam.y() + marker_length_ / 2,
                         center_f_cam.z()};
    cv::Point3d c3_f_cam{center_f_cam.x() - marker_length_ / 2, center_f_cam.y() + marker_length_ / 2,
                         center_f_cam.z()};

    // Project to pixels
    return {}; // TODO
//    return {polar.id,
//            cam_model_.project3dToPixel(c0_f_cam),
//            cam_model_.project3dToPixel(c1_f_cam),
//            cam_model_.project3dToPixel(c2_f_cam),
//            cam_model_.project3dToPixel(c3_f_cam)};
  }

}