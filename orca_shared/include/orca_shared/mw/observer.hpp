#ifndef ORCA_SHARED_MW_OBSERVER_HPP
#define ORCA_SHARED_MW_OBSERVER_HPP

#include "image_geometry/pinhole_camera_model.h"
#include "orca_msgs/msg/observer.hpp"
#include "orca_shared/mw/observation.hpp"
#include "orca_shared/mw/polar_observation.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace mw
{

  class Observer
  {
    orca_msgs::msg::Observer msg_;
    image_geometry::PinholeCameraModel cam_model_;
    tf2::Transform t_base_cam_;
    tf2::Transform t_cam_base_;

  public:

    Observer() = default;

    explicit Observer(const orca_msgs::msg::Observer &msg) :
      msg_{msg}
    {
      cam_model_.fromCameraInfo(msg_.camera_info);
      tf2::fromMsg(msg.cam_f_base, t_base_cam_);
      t_cam_base_ = t_base_cam_.inverse();
    }

    Observer(double marker_length,
             const image_geometry::PinholeCameraModel &cam_model,
             const tf2::Transform &t_base_cam) :
      cam_model_{cam_model},
      t_base_cam_{t_base_cam},
      t_cam_base_{t_base_cam.inverse()}
    {
      msg_.marker_length = marker_length;
      msg_.camera_info = cam_model_.cameraInfo();
      tf2::toMsg(t_base_cam_, msg_.cam_f_base);
    }

    orca_msgs::msg::Observer msg() const
    {
      return msg_;
    }

    double marker_length() const
    {
      return msg_.marker_length;
    }

    double width() const
    {
      return msg_.camera_info.width;
    }

    double height() const
    {
      return msg_.camera_info.height;
    }

    const image_geometry::PinholeCameraModel &cam_model() const
    {
      return cam_model_;
    }

    const tf2::Transform &t_cam_base() const
    {
      return t_cam_base_;
    }

    PolarObservation polar_observation(const Observation &observation);

    Observation observation(const PolarObservation &polar_observation);

    bool operator==(const Observer &that) const
    {
      return msg_ == that.msg_;
    }

    bool operator!=(const Observer &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Observer &v);
  };

}

#endif //ORCA_SHARED_MW_OBSERVER_HPP
