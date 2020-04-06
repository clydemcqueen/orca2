#ifndef ORCA_SHARED_MW_MARKER_HPP
#define ORCA_SHARED_MW_MARKER_HPP

#include "image_geometry/pinhole_camera_model.h"
#include "orca_shared/mw/observation.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Vector3.h"

namespace mw
{

  class Marker
  {
    int id_;
    double marker_length_;
    geometry_msgs::msg::Pose marker_f_map_;
    tf2::Vector3 corner0_f_map_;
    tf2::Vector3 corner1_f_map_;
    tf2::Vector3 corner2_f_map_;
    tf2::Vector3 corner3_f_map_;

  public:

    Marker() = default;

    Marker(const int &id, const double &marker_length, const geometry_msgs::msg::Pose &marker_f_map) :
      id_{id},
      marker_length_{marker_length},
      marker_f_map_{marker_f_map}
    {
      tf2::Transform t_map_marker;
      tf2::fromMsg(marker_f_map, t_map_marker);

      corner0_f_map_ = t_map_marker * tf2::Vector3{-marker_length / 2.f, marker_length / 2.f, 0.f};
      corner1_f_map_ = t_map_marker * tf2::Vector3{marker_length / 2.f, marker_length / 2.f, 0.f};
      corner2_f_map_ = t_map_marker * tf2::Vector3{marker_length / 2.f, -marker_length / 2.f, 0.f};
      corner3_f_map_ = t_map_marker * tf2::Vector3{-marker_length / 2.f, -marker_length / 2.f, 0.f};
    }

    Observation predict_observation(const image_geometry::PinholeCameraModel &cam_model,
                            const tf2::Transform &t_cam_map) const;

    bool operator==(const Marker &that) const
    {
      return id_ == that.id_ &&
             marker_length_ == that.marker_length_ &&
             marker_f_map_ == that.marker_f_map_;
    }

    bool operator!=(const Marker &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Marker &v);
  };

}

#endif //ORCA_SHARED_MW_MARKER_HPP
