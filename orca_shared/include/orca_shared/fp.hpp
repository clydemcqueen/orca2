#ifndef ORCA_SHARED_FP_HPP
#define ORCA_SHARED_FP_HPP

#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "opencv2/core/types.hpp"
#include "orca_msgs/msg/fiducial_pose_stamped.hpp"
#include "orca_msgs/msg/observation.hpp"
#include "orca_shared/geometry.hpp"
#include "rclcpp/time.hpp"

namespace orca
{

  //=====================================================================================
  // Observation -- observation of a marker from a camera
  //
  // Distance and bearing are relative to camera_link, not base_link
  // Bearing measurement is pretty reasonable, but distance measurement is noisy
  //
  // Note: "bearing" is similar to the nautical term "relative bearing" BUT it's the
  // COUNTER-clockwise angle from the heading of the vessel, not the clockwise angle.
  //=====================================================================================

  // Should be Observation::NOT_A_MARKER, but sometimes gives a linker error if < C++17
  constexpr int NOT_A_MARKER = -1;

  struct Observation
  {
    int id{NOT_A_MARKER};                 // Marker ID
    cv::Point2d c0{}, c1{}, c2{}, c3{};   // Corners
    double distance{};                    // Estimated distance to marker
    double bearing{};                     // Bearing to center of marker

    constexpr Observation() = default;

    Observation(const fiducial_vlam_msgs::msg::Observation &msg,
                double marker_length, double hfov, double hres);

    explicit Observation(const orca_msgs::msg::Observation &msg);

    Observation(int _id, const cv::Point2d &_c0, const cv::Point2d &_c1, const cv::Point2d &_c2, const cv::Point2d &_c3,
                double marker_length, double hfov, double hres);

    void estimate_corners(double marker_length, double hfov, double hres, double vres);

    orca_msgs::msg::Observation to_msg() const;

  private:

    void estimate_distance_and_bearing(double marker_length, double hfov, double hres);
  };

  std::ostream &operator<<(std::ostream &os, Observation const &obs);

  //=====================================================================================
  // Observations -- vector of Observation
  //=====================================================================================

  using Observations = std::vector<Observation>;

  //=====================================================================================
  // ObservationStamped -- observation with a timestamp
  //=====================================================================================

  struct ObservationStamped
  {
    rclcpp::Time t{0, 0, RCL_ROS_TIME};
    Observation o{};

    constexpr ObservationStamped() = default;
  };

  std::ostream &operator<<(std::ostream &os, ObservationStamped const &obs);

  //=====================================================================================
  // Fiducial pose (FP) -- pose and observations
  //=====================================================================================

  struct FP
  {
    orca::PoseWithCovariance pose{};
    Observations observations{};

    constexpr FP() = default;

    FP(const fiducial_vlam_msgs::msg::Observations &obs,
       const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg,
       double marker_length, double hfov, double hres);

    explicit FP(const orca_msgs::msg::FiducialPose &msg);

    // True if z is good
    bool good_z() const
    { return pose.good_z(); }

    // True if entire pose is good
    bool good_pose(double good_pose_dist) const;

    // True if there is at least one good observation
    bool has_good_observation(double good_obs_dist) const;

    // Get the observation of a particular marker, return true if found
    bool get_observation(int id, Observation &obs) const;

    // Get the observation of a particular marker, return true if found and the observation is good
    bool get_good_observation(double good_obs_dist, int id, Observation &obs) const;

    // Return the distance of the closest observation
    double closest_observation() const;

    // Get the closest observation and return the distance
    double get_closest_observation(Observation &obs) const;

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

    void set_good_z(double z);
  };

  std::ostream &operator<<(std::ostream &os, FP const &fp);

  //=====================================================================================
  // FPStamped -- pose and observations with timestamp
  //=====================================================================================

  struct FPStamped
  {
    rclcpp::Time t{0, 0, RCL_ROS_TIME};
    FP fp;

    constexpr FPStamped() = default;

    FPStamped(const fiducial_vlam_msgs::msg::Observations &obs,
              const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg,
              double marker_length, double hfov, double hres);

    explicit FPStamped(const orca_msgs::msg::FiducialPoseStamped &msg);

    // Get the observation of a particular marker, return true if found and the observation is good
    bool get_good_observation(double good_obs_dist, int id, ObservationStamped &obs) const;

    // Get the closest observation and return the distance
    double get_closest_observation(ObservationStamped &obs) const;

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

  //=====================================================================================
  // Utilities
  //=====================================================================================

  double closest_observation(const Observations &observations);

  double closest_observation(const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr obs_msg,
                             double marker_length, double hfov, double hres);

  /**
   * Convert vlam msg observations to Orca msg observations by calculating bearing and distance
   *
   * @param vlam_msg In: vector of marker observations from vlam
   * @param orca_msg Out: vector of marker observations with bearing and distance
   * @param marker_length In: marker length
   * @param hfov In: horizontal field of view
   * @param hres In: horizontal resolution
   */
  void vlam_msg_to_orca_msg(const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &vlam_msg,
                            std::vector<orca_msgs::msg::Observation> &orca_msg,
                            double marker_length, double hfov, double hres);

  void vlam_msg_to_orca(const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &vlam_msg,
                        Observations &observations,
                        double marker_length, double hfov, double hres);

  void orca_msg_to_orca(const std::vector<orca_msgs::msg::Observation> &orca_msg, Observations &observations);

  void orca_to_orca_msg(const Observations &observations, std::vector<orca_msgs::msg::Observation> &orca_msg);

} // namespace orca

#endif //ORCA_SHARED_FP_HPP