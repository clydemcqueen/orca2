// Copyright (c) 2020, Clyde McQueen.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "orca_shared/mw/mw.hpp"

#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <vector>

#include "astar/astar.hpp"

namespace mw
{

//=====================================================================================
// Static objects
//=====================================================================================

const Marker Marker::None{NOT_A_MARKER, {}, {}};
const Observation Observation::None{NOT_A_MARKER, {}, {}, {}, {}};
const PolarObservation
PolarObservation::None{NOT_A_MARKER, {std::numeric_limits<double>::max()}, {}};

//=====================================================================================
// Utilities
//=====================================================================================

inline cv::Point3d tf_to_cv(tf2::Vector3 p)
{
  return cv::Point3d(p.x(), p.y(), p.z());
}

// Return true if this point is inside of the rectangle [[0, 0], [2*cx, 2*cy]]
inline bool in_frame(const cv::Point2d & p, double cx, double cy)
{
  return p.x >= 0 && p.x <= 2 * cx && p.y >= 0 && p.y <= 2 * cy;
}

//=====================================================================================
// FiducialPose
//=====================================================================================

int FiducialPose::predict_observations(const Map & map)
{
  auto t_cam_map = observations_.observer().t_cam_base() * pose_.pose().transform().inverse();
  auto model = observations_.observer().camera_model();

  observations_.clear();
  int num_observations = 0;

  for (const auto & marker : map.markers()) {
    Observation observation = marker.predict_observation(model, t_cam_map);
    if (observation != Observation::None) {
      ++num_observations;
      observations_.add(observation);
    }
  }

  return num_observations;
}

//=====================================================================================
// Map
//=====================================================================================

constexpr astar::node_type START_ID = -1;
constexpr astar::node_type DESTINATION_ID = -2;

bool Map::get_waypoints(
  const double & target_z, const double & max_dead_reckon_dist,
  const Pose & start_pose, const Pose & destination_pose, std::vector<Pose> & waypoints) const
{
  waypoints.clear();

  // Create a map of marker ids to poses, including the start and destination pose
  std::map<astar::node_type, Pose> poses;

  // Add start pose to map
  auto start_z_target = start_pose;
  start_z_target.position().z() = target_z;
  poses[START_ID] = start_z_target;

  // Add destination pose to map
  auto destination_z_target = destination_pose;
  destination_z_target.position().z() = target_z;
  poses[DESTINATION_ID] = destination_z_target;

  // Add all of the markers to the map
  for (size_t i = 0; i < msg_.ids.size(); ++i) {
    Pose pose{msg_.poses[i].pose};
    pose.position().z() = target_z;
    poses[msg_.ids[i]] = pose;
  }

  // Enumerate all edges between markers that are < cxt_.pose_plan_max_dead_reckon_dist_
  std::vector<astar::Edge> short_paths{};
  for (auto i = poses.begin(); i != poses.end(); ++i) {
    for (auto j = std::next(i); j != poses.end(); ++j) {
      auto distance = i->second.position().distance_xy(j->second.position());
      if (distance < max_dead_reckon_dist) {
        short_paths.emplace_back(i->first, j->first, distance);
      }
    }
  }

  // Create an A* solver that we can use to navigate between markers that are further away
  auto solver = std::make_shared<astar::Solver>(short_paths,
      [&](astar::node_type a, astar::node_type b) -> double
      {
        return poses[a].position().distance_xy(poses[b].position());
      });

  // Find the shortest path from the start pose to the destination pose through markers
  std::vector<astar::node_type> path{};
  if (solver->find_shortest_path(START_ID, DESTINATION_ID, path)) {
    for (auto marker : path) {
      waypoints.push_back(poses[marker]);
    }
    return true;
  } else {
    std::cout << "A* failed to find a path through the markers" << std::endl;
    return false;
  }
}

//=====================================================================================
// Marker
//=====================================================================================

Observation Marker::predict_observation(
  const image_geometry::PinholeCameraModel & cam_model,
  const tf2::Transform & t_cam_map) const
{
  // Camera frame: x right, y down, z forward

  // Transform corners from map frame to camera frame
  auto corner0_f_cam = t_cam_map * corner0_f_map_;
  auto corner1_f_cam = t_cam_map * corner1_f_map_;
  auto corner2_f_cam = t_cam_map * corner2_f_map_;
  auto corner3_f_cam = t_cam_map * corner3_f_map_;

  // Ignore markers that are behind the camera
  if (corner0_f_cam.z() < 0 || corner1_f_cam.z() < 0 || corner2_f_cam.z() < 0 ||
    corner3_f_cam.z() < 0)
  {
    return Observation::None;
  }

  // TODO(clyde): ignore markers that are not facing the camera

  // Project corners onto the image plane
  auto c0 = cam_model.project3dToPixel(tf_to_cv(corner0_f_cam));
  auto c1 = cam_model.project3dToPixel(tf_to_cv(corner1_f_cam));
  auto c2 = cam_model.project3dToPixel(tf_to_cv(corner2_f_cam));
  auto c3 = cam_model.project3dToPixel(tf_to_cv(corner3_f_cam));

  // Ignore markers that are outside of the visible frame
  if (!in_frame(c0, cam_model.cx(), cam_model.cy()) ||
    !in_frame(c1, cam_model.cx(), cam_model.cy()) ||
    !in_frame(c2, cam_model.cx(), cam_model.cy()) ||
    !in_frame(c3, cam_model.cx(), cam_model.cy()))
  {
    return Observation::None;
  }

  return Observation{id_, c0, c1, c2, c3};
}

//=====================================================================================
// Observer
//=====================================================================================

void Observer::convert(const Observation & observation, PolarObservation & polar_observation)
{
  assert(marker_length() != 0 && width() != 0 && height() != 0);

  // Use the camera model to project the 2d corner points to 3d rays
  // The z value is always 1.0
  auto model = camera_model();
  auto ray0 = model.projectPixelTo3dRay(observation.c0());
  auto ray1 = model.projectPixelTo3dRay(observation.c1());
  auto ray2 = model.projectPixelTo3dRay(observation.c2());
  auto ray3 = model.projectPixelTo3dRay(observation.c3());

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
  auto scale_factor = marker_length() / longest_side;

  // Find the center of the marker
  tf2::Vector3 center_f_cam{(ray0.x + ray1.x + ray2.x + ray3.x) * scale_factor / 4,
    (ray0.y + ray1.y + ray2.y + ray3.y) * scale_factor / 4,
    scale_factor};

  // Transform the center point from the camera frame to the base frame
  auto center_f_base = t_base_cam() * center_f_cam;

  // Calculate distance and bearing
  polar_observation = {observation.id(),
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
void Observer::convert(const PolarObservation & polar_observation, Observation & observation)
{
  assert(marker_length() != 0 && width() != 0 && height() != 0);

  // Find the center point
  tf2::Vector3 center_f_base{std::cos(polar_observation.bearing()) * polar_observation.distance(),
    std::sin(polar_observation.bearing()) * polar_observation.distance(),
    0};

  // Transform the center point from the base frame to the camera frame
  auto center_f_cam = t_cam_base() * center_f_base;

  // Build corners in the camera frame
  cv::Point3d
    c0_f_cam{center_f_cam.x() - marker_length() / 2, center_f_cam.y() - marker_length() / 2,
    center_f_cam.z()};
  cv::Point3d
    c1_f_cam{center_f_cam.x() + marker_length() / 2, center_f_cam.y() - marker_length() / 2,
    center_f_cam.z()};
  cv::Point3d
    c2_f_cam{center_f_cam.x() + marker_length() / 2, center_f_cam.y() + marker_length() / 2,
    center_f_cam.z()};
  cv::Point3d
    c3_f_cam{center_f_cam.x() - marker_length() / 2, center_f_cam.y() + marker_length() / 2,
    center_f_cam.z()};

  // Project to pixels
  auto model = camera_model();
  observation = {polar_observation.id(),
    model.project3dToPixel(c0_f_cam),
    model.project3dToPixel(c1_f_cam),
    model.project3dToPixel(c2_f_cam),
    model.project3dToPixel(c3_f_cam)};
}

//=====================================================================================
// operator<<
//=====================================================================================

std::ostream & operator<<(std::ostream & os, const Acceleration & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.x() << ", " <<
         v.y() << ", " <<
         v.z() << ", " <<
         v.yaw() << "}";
}

std::ostream & operator<<(std::ostream & os, const AccelerationBody & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.forward() << ", " <<
         v.strafe() << ", " <<
         v.vertical() << ", " <<
         v.yaw() << "}";
}

std::ostream & operator<<(std::ostream & os, Efforts const & e)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
            e.forward() << ", " <<
            e.strafe() << ", " <<
            e.vertical() << ", " <<
            e.yaw() << "}";
}

std::ostream & operator<<(std::ostream & os, const FiducialPose & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.observations() << ", " <<
         v.pose() << "}";
}

std::ostream & operator<<(std::ostream & os, const FiducialPoseStamped & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.header() << ", " <<
         v.fp() << "}";
}

std::ostream & operator<<(std::ostream & os, const Header & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.t().nanoseconds() << ", " <<
         v.frame_id() << "}";
}

std::ostream & operator<<(std::ostream & os, const Map & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.msg_.poses.size() << "}";
}

std::ostream & operator<<(std::ostream & os, const Marker & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.id_ << ", " <<
         v.corner0_f_map_ << ", " <<
         v.corner1_f_map_ << ", " <<
         v.corner2_f_map_ << ", " <<
         v.corner3_f_map_ << "}";
}

std::ostream & operator<<(std::ostream & os, const MissionState & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.plan() << ", " <<
         v.twist() << "}";
}

std::ostream & operator<<(std::ostream & os, const Observation & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.id() << ", {" <<
         v.c0().x << ", " << v.c0().y << "}, {" <<
         v.c1().x << ", " << v.c1().y << "}, {" <<
         v.c2().x << ", " << v.c2().y << "}, {" <<
         v.c3().x << ", " << v.c3().y << "}}";
}

std::ostream & operator<<(std::ostream & os, ObservationStamped const & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.header() << ", " <<
         v.observation() << "}";
}

std::ostream & operator<<(std::ostream & os, const Observations & v)
{
  os << std::fixed << std::setprecision(3) << "{" << v.observer() << ", {";
  for (const auto & item : v.observations_) {
    os << item << ", ";
  }
  os << "}, {";
  for (const auto & item : v.polar_observations_) {
    os << item << ", ";
  }
  os << "}";
  return os;
}

std::ostream & operator<<(std::ostream & os, const Observer & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.marker_length() << ", " <<
         v.width() << ", " <<
         v.height() << ", " <<
         mw::Pose(v.msg_.cam_f_base) << "}";
}

std::ostream & operator<<(std::ostream & os, const Point & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.x() << ", " <<
         v.y() << ", " <<
         v.z() << "}";
}

std::ostream & operator<<(std::ostream & os, const Pose & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.position() << ", " <<
         v.orientation() << "}";
}

std::ostream & operator<<(std::ostream & os, const PoseStamped & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.header() << ", " <<
         v.pose() << "}";
}

std::ostream & operator<<(std::ostream & os, const PoseWithCovariance & v)
{
  os << v.pose();
#if 0
  os << ", {";
  for (const auto & item : v.covariance_) {
    os << item << ", ";
  }
  os << "}";
#endif
  return os;
}

std::ostream & operator<<(std::ostream & os, PoseWithCovarianceStamped const & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.header() << ", " <<
         v.pose() << "}";
}

std::ostream & operator<<(std::ostream & os, PolarObservation const & v)
{
  os << std::fixed << std::setprecision(3) << "{";
  if (v.id() == NOT_A_MARKER) {
    os << "NOT_A_MARKER, max double";
  } else {
    os << v.id() << ", " << v.distance();
  }
  return os << ", " << v.bearing() << "}";
}

std::ostream & operator<<(std::ostream & os, PolarObservationStamped const & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.header() << ", " <<
         v.observation() << "}";
}

std::ostream & operator<<(std::ostream & os, const Quaternion & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.roll() << ", " <<
         v.pitch() << ", " <<
         v.yaw() << "}";
}

std::ostream & operator<<(std::ostream & os, const Target & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.id() << ", " <<
         v.pose() << "}";
}

std::ostream & operator<<(std::ostream & os, const Twist & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.x() << ", " <<
         v.y() << ", " <<
         v.z() << ", " <<
         v.yaw() << "}";
}

std::ostream & operator<<(std::ostream & os, const TwistBody & v)
{
  return os << std::fixed << std::setprecision(3) << "{" <<
         v.forward() << ", " <<
         v.strafe() << ", " <<
         v.vertical() << ", " <<
         v.yaw() << "}";
}

}  // namespace mw
