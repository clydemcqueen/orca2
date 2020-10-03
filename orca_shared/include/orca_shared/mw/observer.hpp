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

#ifndef ORCA_SHARED__MW__OBSERVER_HPP_
#define ORCA_SHARED__MW__OBSERVER_HPP_

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

  explicit Observer(const orca_msgs::msg::Observer & msg)
  : msg_{msg}
  {
    init();
  }

  // From sensor_msgs + geometry_msgs
  Observer(
    const double & marker_length,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const geometry_msgs::msg::Pose & cam_f_base)
  {
    msg_.marker_length = marker_length;
    msg_.camera_info = camera_info;
    msg_.cam_f_base = cam_f_base;

    init();
  }

  Observer(
    const double & marker_length,
    const image_geometry::PinholeCameraModel & camera_model,
    const tf2::Transform & t_base_cam)
  : camera_model_{camera_model},
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

  const sensor_msgs::msg::CameraInfo & camera_info() const
  {
    return msg_.camera_info;
  }

  const geometry_msgs::msg::Pose & cam_f_base() const
  {
    return msg_.cam_f_base;
  }

  const image_geometry::PinholeCameraModel & camera_model() const
  {
    return camera_model_;
  }

  const tf2::Transform & t_base_cam() const
  {
    return t_base_cam_;
  }

  const tf2::Transform & t_cam_base() const
  {
    return t_cam_base_;
  }

  void convert(const Observation & observation, PolarObservation & polar_observation);

  void convert(const PolarObservation & polar_observation, Observation & observation);

  bool operator==(const Observer & that) const
  {
    return msg_ == that.msg_;
  }

  bool operator!=(const Observer & that) const
  {
    return !(*this == that);
  }

  friend std::ostream & operator<<(std::ostream & os, const Observer & v);
};

}  // namespace mw

#endif  // ORCA_SHARED__MW__OBSERVER_HPP_
