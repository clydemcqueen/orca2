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

  public:

    Observer() = default;

    explicit Observer(const orca_msgs::msg::Observer &msg) :
      msg_{msg}
    {}

    // From sensor_msgs + geometry_msgs
    Observer(const double &marker_length,
             const sensor_msgs::msg::CameraInfo &camera_info,
             const geometry_msgs::msg::Pose &cam_f_base)
    {
      msg_.marker_length = marker_length;
      msg_.camera_info = camera_info;
      msg_.cam_f_base = cam_f_base;
    }

    Observer(const double &marker_length,
             const image_geometry::PinholeCameraModel &cam_model,
             const tf2::Transform &t_base_cam)
    {
      msg_.marker_length = marker_length;
      msg_.camera_info = cam_model.cameraInfo();
      tf2::toMsg(t_base_cam, msg_.cam_f_base);
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

    const sensor_msgs::msg::CameraInfo &camera_info() const
    {
      return msg_.camera_info;
    }

    const geometry_msgs::msg::Pose &cam_f_base() const
    {
      return msg_.cam_f_base;
    }

    image_geometry::PinholeCameraModel camera_model() const
    {
      image_geometry::PinholeCameraModel result;
      result.fromCameraInfo(msg_.camera_info);
      return result;
    }

    tf2::Transform t_cam_base() const
    {
      tf2::Transform result;
      tf2::fromMsg(cam_f_base(), result);
      return result;
    }

    tf2::Transform t_base_cam() const
    {
      return t_cam_base().inverse();
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
