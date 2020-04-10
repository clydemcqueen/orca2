#ifndef ORCA_SHARED_MW_MARKER_HPP
#define ORCA_SHARED_MW_MARKER_HPP

#include "image_geometry/pinhole_camera_model.h"
#include "orca_shared/mw/observation.hpp"
#include "orca_shared/mw/target.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Vector3.h"

namespace mw
{

  class Marker
  {
    int id_{};
    double marker_length_{};
    Pose pose_;
    tf2::Vector3 corner0_f_map_; // TODO mw::Point
    tf2::Vector3 corner1_f_map_; // TODO mw::Point
    tf2::Vector3 corner2_f_map_; // TODO mw::Point
    tf2::Vector3 corner3_f_map_; // TODO mw::Point

  public:

    static const Marker None;

    Marker() = default;

    Marker(const int &id, const double &marker_length, const Pose &pose) :
      id_{id},
      marker_length_{marker_length},
      pose_{pose}
    {
      tf2::Transform t_map_marker = pose.transform();
      corner0_f_map_ = t_map_marker * tf2::Vector3{-marker_length / 2.f, marker_length / 2.f, 0.f};
      corner1_f_map_ = t_map_marker * tf2::Vector3{marker_length / 2.f, marker_length / 2.f, 0.f};
      corner2_f_map_ = t_map_marker * tf2::Vector3{marker_length / 2.f, -marker_length / 2.f, 0.f};
      corner3_f_map_ = t_map_marker * tf2::Vector3{-marker_length / 2.f, -marker_length / 2.f, 0.f};
    }

    int id() const
    {
      return id_;
    }

    double marker_length() const
    {
      return marker_length_;
    }

    Observation predict_observation(const image_geometry::PinholeCameraModel &cam_model,
                            const tf2::Transform &t_cam_map) const;

    // Floor == true: markers must be on the floor facing up, and there must be a down-facing camera
    // Floor == false: markers must be on the wall, and there must be a forward-facing camera
    Target target(const double &target_z, const double &target_dist, const bool &floor) const
    {
      Pose target_pose = pose_;

      // Set plan.z from parameters
      target_pose.z() = target_z;

      if (!floor) {
        // Target is in front of the marker
        target_pose.x() += sin(target_pose.yaw()) * target_dist;
        target_pose.y() -= cos(target_pose.yaw()) * target_dist;

        // Face the marker to get a good pose
        target_pose.yaw(target_pose.yaw() + M_PI_2);
      }

      return {id(), target_pose};
    }

    bool operator==(const Marker &that) const
    {
      return id_ == that.id_ &&
             marker_length_ == that.marker_length_ &&
             pose_ == that.pose_;
    }

    bool operator!=(const Marker &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Marker &v);
  };

}

#endif //ORCA_SHARED_MW_MARKER_HPP
