#ifndef ORCA_BASE_FP_HPP
#define ORCA_BASE_FP_HPP

#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "opencv2/core/types.hpp"
#include "orca_shared/geometry.hpp"
#include "rclcpp/time.hpp"

namespace orca_base
{

  //=====================================================================================
  // Observation -- observation of a marker from a camera
  // Distance and yaw are relative to camera_link, not base_link
  // Yaw measurement is pretty reasonable, but distance measurement is noisy
  //=====================================================================================

  // Should be Observation::NOT_A_MARKER, but sometimes gives a linker error if < C++17
  constexpr int NOT_A_MARKER = -1;

  struct Observation
  {
    int id{NOT_A_MARKER};         // Marker ID
    cv::Point2d c0, c1, c2, c3;   // Corners
    double distance{0};           // Estimated distance to marker
    double yaw{0};                // Yaw to center of marker

    Observation()
    = default;

    // Construct from vlam observation and z
    Observation(const fiducial_vlam_msgs::msg::Observation &msg,
                double marker_length, double hfov, double hres);

    // Construct from id, corners and z
    Observation(int _id, const cv::Point2d &_c0, const cv::Point2d &_c1, const cv::Point2d &_c2, const cv::Point2d &_c3,
                double marker_length, double hfov, double hres);

    void estimate_corners(double marker_length, double hfov, double hres, double vres);

  private:

    void estimate_distance_and_yaw(double marker_length, double hfov, double hres);
  };

  std::ostream &operator<<(std::ostream &os, Observation const &obs);

  //=====================================================================================
  // ObservationStamped -- observation with a timestamp
  //=====================================================================================

  struct ObservationStamped
  {
    rclcpp::Time t{0, 0, RCL_ROS_TIME};
    Observation o;
  };

  std::ostream &operator<<(std::ostream &os, ObservationStamped const &obs);

  //=====================================================================================
  // Fiducial pose (FP) -- pose and observations
  //=====================================================================================

  struct FP
  {
    orca::PoseWithCovariance pose;
    std::vector<Observation> observations;

    // True if z is good
    bool good_z() const
    { return pose.good_z(); }

    // True if entire pose is good
    bool good_pose(double good_pose_dist) const;

    // True if there is at least one good observation
    bool good_obs() const
    { return !observations.empty(); }

    // True if there is a good observation of a particular marker
    bool good_obs(int id) const;

    // Get the observation of a particular marker, return true if successful
    bool good_obs(int id, Observation &obs) const;

    // Return the distance of the closest observation
    double closest_obs() const;

    // Get the closest observation and return the distance
    double closest_obs(Observation &obs) const;

    // Could be a constructor
    void from_msgs(const fiducial_vlam_msgs::msg::Observations &obs,
                   const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg, double z, double marker_length,
                   double hfov, double hres);

    // XY distance between 2 poses
    double distance_xy(const FP &that) const
    {
      return pose.pose.distance_xy(that.pose.pose);
    }

    // Z distance between 2 poses
    double distance_z(const FP &that) const
    {
      return pose.pose.distance_z(that.pose.pose);
    }

    // Yaw distance between 2 poses
    double distance_yaw(const FP &that) const
    {
      return pose.pose.distance_yaw(that.pose.pose);
    }
  };

  std::ostream &operator<<(std::ostream &os, FP const &fp);

  //=====================================================================================
  // FPStamped -- pose and observations with timestamp
  //=====================================================================================

  struct FPStamped
  {
    rclcpp::Time t{0, 0, RCL_ROS_TIME};
    FP fp;

    // Get the observation of a particular marker, return true if successful
    bool good_obs(int id, ObservationStamped &obs) const;

    // Get the closest observation and return the distance
    double closest_obs(ObservationStamped &obs) const;

    // Could be a constructor
    void from_msgs(const fiducial_vlam_msgs::msg::Observations &obs,
                   const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg, double z, double marker_length,
                   double hfov, double hres);

    void add_to_path(nav_msgs::msg::Path &path) const;

    // XY distance between 2 poses
    double distance_xy(const FPStamped &that) const
    {
      return fp.pose.pose.distance_xy(that.fp.pose.pose);
    }

    // Z distance between 2 poses
    double distance_z(const FPStamped &that) const
    {
      return fp.pose.pose.distance_z(that.fp.pose.pose);
    }

    // Yaw distance between 2 poses
    double distance_yaw(const FPStamped &that) const
    {
      return fp.pose.pose.distance_yaw(that.fp.pose.pose);
    }
  };

  std::ostream &operator<<(std::ostream &os, FPStamped const &fp);

}

#endif //ORCA_BASE_FP_HPP