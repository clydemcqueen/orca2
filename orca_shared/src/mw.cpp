#include "orca_shared/mw/mw.hpp"

#include <iomanip>

namespace mw
{

  //=====================================================================================
  // Static objects
  //=====================================================================================

  const Observation Observation::None{NOT_A_MARKER, {}, {}, {}, {}};
  const PolarObservation PolarObservation::None{NOT_A_MARKER, {std::numeric_limits<double>::max()}, {}};

  //=====================================================================================
  // Utilities
  //=====================================================================================

  inline cv::Point3d tf_to_cv(tf2::Vector3 p)
  {
    return cv::Point3d(p.x(), p.y(), p.z());
  }

  // Return true if this point is inside of the rectangle [[0, 0], [2*cx, 2*cy]]
  inline bool in_frame(const cv::Point2d &p, double cx, double cy)
  {
    return p.x >= 0 && p.x <= 2 * cx && p.y >= 0 && p.y <= 2 * cy;
  }

  //=====================================================================================
  // Map
  //=====================================================================================

  int Map::predict_observations(const Pose &base_f_map, Observations &observations)
  {
    auto t_cam_map = observations.observer().t_cam_base() * base_f_map.transform().inverse();
    auto model = observations.observer().camera_model();

    observations.clear();
    int num_observations = 0;

    for (const auto &marker : markers_) {
      Observation observation = marker.predict_observation(model, t_cam_map);
      if (observation != Observation::None) {
        ++num_observations;
        observations.add(observation);
      }
    }

    return num_observations;
  }

  //=====================================================================================
  // Marker
  //=====================================================================================

  Observation Marker::predict_observation(const image_geometry::PinholeCameraModel &cam_model,
                                          const tf2::Transform &t_cam_map) const
  {
    // Camera frame: x right, y down, z forward

    // Transform corners from map frame to camera frame
    auto corner0_f_cam = t_cam_map * corner0_f_map_;
    auto corner1_f_cam = t_cam_map * corner1_f_map_;
    auto corner2_f_cam = t_cam_map * corner2_f_map_;
    auto corner3_f_cam = t_cam_map * corner3_f_map_;

    // Ignore markers that are behind the camera
    if (corner0_f_cam.z() < 0 || corner1_f_cam.z() < 0 || corner2_f_cam.z() < 0 || corner3_f_cam.z() < 0) {
      return Observation::None;
    }

    // Ignore markers that are not facing the camera TODO

    // Project corners onto the image plane
    auto c0 = cam_model.project3dToPixel(tf_to_cv(corner0_f_cam));
    auto c1 = cam_model.project3dToPixel(tf_to_cv(corner1_f_cam));
    auto c2 = cam_model.project3dToPixel(tf_to_cv(corner2_f_cam));
    auto c3 = cam_model.project3dToPixel(tf_to_cv(corner3_f_cam));

    // Ignore markers that are outside of the visible frame
    if (!in_frame(c0, cam_model.cx(), cam_model.cy()) ||
        !in_frame(c1, cam_model.cx(), cam_model.cy()) ||
        !in_frame(c2, cam_model.cx(), cam_model.cy()) ||
        !in_frame(c3, cam_model.cx(), cam_model.cy())) {
      return Observation::None;
    }

    return Observation{id_, c0, c1, c2, c3};
  }

  //=====================================================================================
  // Observer
  //=====================================================================================

  PolarObservation Observer::polar_observation(const Observation &observation)
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
    return {observation.id(),
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
  Observation Observer::observation(const PolarObservation &polar_observation)
  {
    assert(marker_length() != 0 && width() != 0 && height() != 0);

    // Find the center point
    tf2::Vector3 center_f_base{std::cos(polar_observation.bearing()) * polar_observation.distance(),
                               std::sin(polar_observation.bearing()) * polar_observation.distance(),
                               0};

    // Transform the center point from the base frame to the camera frame
    auto center_f_cam = t_cam_base() * center_f_base;

    // Build corners in the camera frame
    cv::Point3d c0_f_cam{center_f_cam.x() - marker_length() / 2, center_f_cam.y() - marker_length() / 2,
                         center_f_cam.z()};
    cv::Point3d c1_f_cam{center_f_cam.x() + marker_length() / 2, center_f_cam.y() - marker_length() / 2,
                         center_f_cam.z()};
    cv::Point3d c2_f_cam{center_f_cam.x() + marker_length() / 2, center_f_cam.y() + marker_length() / 2,
                         center_f_cam.z()};
    cv::Point3d c3_f_cam{center_f_cam.x() - marker_length() / 2, center_f_cam.y() + marker_length() / 2,
                         center_f_cam.z()};

    // Project to pixels
    auto model = camera_model();
    return {polar_observation.id(),
            model.project3dToPixel(c0_f_cam),
            model.project3dToPixel(c1_f_cam),
            model.project3dToPixel(c2_f_cam),
            model.project3dToPixel(c3_f_cam)};
  }

  //=====================================================================================
  // operator<<
  //=====================================================================================

  std::ostream &operator<<(std::ostream &os, Acceleration const &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.x() << ", " << v.y() << ", " << v.z() << ", " << v.yaw() << "}";
  }

  std::ostream &operator<<(std::ostream &os, const FiducialPose &v)
  {
    return os << std::fixed << std::setprecision(3)
              << ", " << v.observations()
              << ", " << v.pose() << "}";
  }

  std::ostream &operator<<(std::ostream &os, const FiducialPoseStamped &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.header() << ", " << v.fp() << "}";
  }

  std::ostream &operator<<(std::ostream &os, const Header &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.stamp().nanoseconds() << ", " << v.frame_id() << "}";
  }

  std::ostream &operator<<(std::ostream &os, const Map &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.msg_.poses.size() << "}";
  }

  std::ostream &operator<<(std::ostream &os, const Marker &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.id_
              << ", " << v.corner0_f_map_
              << ", " << v.corner1_f_map_
              << ", " << v.corner2_f_map_
              << ", " << v.corner3_f_map_ << "}";
  }

  std::ostream &operator<<(std::ostream &os, const Observation &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.id()
              << ", {" << v.c0().x << ", " << v.c0().y
              << "}, {" << v.c1().x << ", " << v.c1().y
              << "}, {" << v.c2().x << ", " << v.c2().y
              << "}, {" << v.c3().x << ", " << v.c3().y << "}}";
  }

  std::ostream &operator<<(std::ostream &os, const Observations &v)
  {
    os << std::fixed << std::setprecision(3) << "{" << v.observer() << ", {";
    for (const auto &item : v.observations_) {
      os << item << ", ";
    }
    os << "}, {";
    for (const auto &item : v.polar_observations_) {
      os << item << ", ";
    }
    os << "}";
    return os;
  }

  std::ostream &operator<<(std::ostream &os, const Observer &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{marker_length: " << v.marker_length()
              << ", width: " << v.width()
              << ", height: " << v.height() << "}";
  }

  std::ostream &operator<<(std::ostream &os, const Point &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.x() << ", " << v.y() << ", " << v.z() << "}";
  }

  std::ostream &operator<<(std::ostream &os, const Pose &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.position() << ", " << v.orientation() << "}";
  }

  std::ostream &operator<<(std::ostream &os, const PoseWithCovariance &v)
  {
    os << v.pose();
#if 0
    os << ", {";
    for (const auto &item : v.covariance_) {
      os << item << ", ";
    }
    os << "}";
#endif
    return os;
  }

  std::ostream &operator<<(std::ostream &os, PoseWithCovarianceStamped const &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.header() << ", " << v.pose()  << "}";
  }

  std::ostream &operator<<(std::ostream &os, PolarObservation const &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.id() << ", " << v.distance() << ", " << v.bearing() << "}";
  }

  std::ostream &operator<<(std::ostream &os, const Quaternion &v)
  {
    return os << std::fixed << std::setprecision(3)
              << "{" << v.roll() << ", " << v.pitch() << ", " << v.yaw() << "}";
  }

//    Pose project(const rclcpp::Duration &d, const mw::Twist &v0, const mw::Acceleration &a) const
//    {
//      double dt = d.seconds();
//      Pose result;
//      result.x = x + v0.x * dt + 0.5 * a.x * dt * dt;
//      result.y = y + v0.y * dt + 0.5 * a.y * dt * dt;
//      result.z = z + v0.z * dt + 0.5 * a.z * dt * dt;
//      result.yaw = orca::norm_angle(yaw + v0.yaw * dt + 0.5 * a.yaw * dt * dt);
//      return result;
//    }

}