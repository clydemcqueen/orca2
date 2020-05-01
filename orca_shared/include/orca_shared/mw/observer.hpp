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
    image_geometry::PinholeCameraModel camera_model_;
    tf2::Transform t_base_cam_;
    tf2::Transform t_cam_base_;

    void init()
    {
      camera_model_.fromCameraInfo(msg_.camera_info);
      tf2::fromMsg(msg_.cam_f_base, t_base_cam_);
      t_cam_base_ = t_base_cam_.inverse();
    }

  public:

    Observer() = default;

    explicit Observer(const orca_msgs::msg::Observer &msg) :
      msg_{msg}
    {
      init();
    }

    // From sensor_msgs + geometry_msgs
    Observer(const double &marker_length,
             const sensor_msgs::msg::CameraInfo &camera_info,
             const geometry_msgs::msg::Pose &cam_f_base)
    {
      msg_.marker_length = marker_length;
      msg_.camera_info = camera_info;
      msg_.cam_f_base = cam_f_base;

      init();
    }

    Observer(const double &marker_length,
             const image_geometry::PinholeCameraModel &camera_model,
             const tf2::Transform &t_base_cam) :
      camera_model_{camera_model},
      t_base_cam_{t_base_cam},
      t_cam_base_{t_base_cam.inverse()}
    {
      msg_.marker_length = marker_length;
      msg_.camera_info = camera_model.cameraInfo();
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

    const image_geometry::PinholeCameraModel &camera_model() const
    {
      return camera_model_;
    }

    const tf2::Transform &t_base_cam() const
    {
      return t_base_cam_;
    }

    const tf2::Transform &t_cam_base() const
    {
      return t_cam_base_;
    }

    void convert(const Observation &observation, PolarObservation &polar_observation);

    void convert(const PolarObservation &polar_observation, Observation &observation);

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
