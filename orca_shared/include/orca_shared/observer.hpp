#ifndef ORCA_SHARED_OBSERVER_HPP
#define ORCA_SHARED_OBSERVER_HPP

#include "image_geometry/pinhole_camera_model.h"
#include "tf2/LinearMath/Transform.h"

#include "orca_shared/fp.hpp"

namespace orca
{

  //=====================================================================================
  // Utilities
  //=====================================================================================

  tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose &pose);

  geometry_msgs::msg::Pose transform_to_pose(const tf2::Transform &transform);

  //=====================================================================================
  // Polar -- observation of a marker from a camera in polar coordinates from base_link
  //
  // Bearing measurement is generally good, but distance measurement can be noisy
  //
  // Note: "bearing" is similar to the nautical term "relative bearing" BUT it's the
  // counter-clockwise angle from the heading of the vessel, not the clockwise angle.
  //=====================================================================================

  struct Polar
  {
    int id{NOT_A_MARKER}; // Marker ID
    double distance{std::numeric_limits<double>::max()};    // Distance from base_link to marker
    double bearing{};                                       // Bearing from base_link to marker

    constexpr Polar() = default;

    Polar(int _id, double _distance, double _bearing) :
      id{_id}, distance{_distance}, bearing{_bearing}
    {}
  };

  std::ostream &operator<<(std::ostream &os, Polar const &o);

  //=====================================================================================
  // PolarStamped
  // TODO I think that this is required by the segments
  //=====================================================================================

  struct PolarStamped
  {
    rclcpp::Time t{0, 0, RCL_ROS_TIME};
    // TODO frame_id
    Polar p{};
  };

  std::ostream &operator<<(std::ostream &os, PolarStamped const &o);

  //=====================================================================================
  // Polars
  //=====================================================================================

  struct Polars
  {
    std::vector<Polar> v{};

    constexpr Polars() = default;

    Polar get(int id) const;

    Polar closest() const;
  };

  std::ostream &operator<<(std::ostream &os, Polars const &o);

  //=====================================================================================
  // Observer -- information about the camera that has made the observations
  //=====================================================================================

  class Observer
  {
    double marker_length_;
    image_geometry::PinholeCameraModel cam_model_;
    tf2::Transform t_base_cam_;
    tf2::Transform t_cam_base_;

  public:

    Observer(const orca_msgs::msg::FiducialPose &msg);

    Observer(double marker_length,
             const image_geometry::PinholeCameraModel &cam_model,
             const tf2::Transform &t_base_cam) :
      marker_length_{marker_length},
      cam_model_{cam_model},
      t_base_cam_{t_base_cam},
      t_cam_base_{t_base_cam.inverse()}
    {}

    double marker_length() const
    { return marker_length_; }

    image_geometry::PinholeCameraModel cam_model() const
    { return cam_model_; }

    tf2::Transform t_base_cam() const
    { return t_base_cam_; }

    tf2::Transform t_cam_base() const
    { return t_cam_base_; }

    Polar get_polar(const Observation &observation);

    Observation get_observation(const Polar &polar);
  };

} // namespace orca

#endif //ORCA_SHARED_OBSERVER_HPP