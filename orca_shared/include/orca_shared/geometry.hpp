#ifndef ORCA_SHARED_GEOMETRY_HPP
#define ORCA_SHARED_GEOMETRY_HPP

#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"

#include "orca_msgs/msg/efforts.hpp"

#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "opencv2/core/types.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// 4 DoF geometry and motion structs

namespace orca
{

  //=====================================================================================
  // Pose in world frame
  //=====================================================================================

  struct Pose
  {
    double x;
    double y;
    double z;
    double yaw;

    constexpr Pose() : x{0}, y{0}, z{0}, yaw{0}
    {}

    geometry_msgs::msg::Pose to_msg() const
    {
      geometry_msgs::msg::Pose msg;

      msg.position.x = x;
      msg.position.y = y;
      msg.position.z = z;

      // Yaw to quaternion
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      msg.orientation = tf2::toMsg(q);

      return msg;
    }

    void from_msg(const geometry_msgs::msg::Pose &msg)
    {
      // Convert ROS Pose to Orca2 Pose by dropping roll & pitch
      x = msg.position.x;
      y = msg.position.y;
      z = msg.position.z;
      yaw = get_yaw(msg.orientation);
    }

    // Distance between 2 poses on the xy plane
    double distance_xy(const Pose &that) const
    {
      return std::hypot(x - that.x, y - that.y);
    }

    double distance_xy(double _x, double _y) const
    {
      return std::hypot(x - _x, y - _y);
    }

    double distance_xy(const nav_msgs::msg::Odometry &msg) const
    {
      return std::hypot(x - msg.pose.pose.position.x, y - msg.pose.pose.position.y);
    }

    double distance_xy(const geometry_msgs::msg::Pose &msg) const
    {
      return std::hypot(x - msg.position.x, y - msg.position.y);
    }

    // Z distance between 2 poses
    double distance_z(const Pose &that) const
    {
      return std::abs(z - that.z);
    }

    double distance_z(const nav_msgs::msg::Odometry &msg) const
    {
      return std::abs(z - msg.pose.pose.position.z);
    }

    // Yaw distance between 2 poses
    double distance_yaw(const Pose &that) const
    {
      return std::abs(norm_angle(yaw - that.yaw));
    }

    double distance_yaw(const nav_msgs::msg::Odometry &msg) const
    {
      return std::abs(norm_angle(yaw - get_yaw(msg.pose.pose.orientation)));
    }
  };

  std::ostream &operator<<(std::ostream &os, Pose const &pose);

  //=====================================================================================
  // PoseStamped
  //=====================================================================================

  struct PoseStamped
  {
    rclcpp::Time t;
    Pose pose;

    geometry_msgs::msg::PoseStamped to_msg() const
    {
      geometry_msgs::msg::PoseStamped msg;

      msg.header.stamp = t;
      msg.pose = pose.to_msg();

      return msg;
    }

    void from_msg(const geometry_msgs::msg::PoseStamped &msg)
    {
      t = msg.header.stamp;
      pose.from_msg(msg.pose);
    }

    void from_msg(const nav_msgs::msg::Odometry &msg)
    {
      t = msg.header.stamp;
      pose.from_msg(msg.pose.pose);
    }

    void from_msg(geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
      t = msg.header.stamp;
      pose.from_msg(msg.pose.pose);
    }

    void add_to_path(nav_msgs::msg::Path &path) const
    {
      auto msg = to_msg();
      msg.header.frame_id = path.header.frame_id;
      path.poses.push_back(msg);
    }
  };

  //=====================================================================================
  // PoseWithCovariance
  //=====================================================================================

  struct PoseWithCovariance
  {
    Pose pose;

    // Don't store the full 6x6 covariance, just store 4 bits of data
    bool x_valid;
    bool y_valid;
    bool z_valid;
    bool yaw_valid;

    bool good_pose() const
    { return x_valid && y_valid && z_valid && yaw_valid; }

    bool good_z() const
    { return z_valid; }

    void from_msg(const geometry_msgs::msg::PoseWithCovariance &pose_with_covariance);
  };

  //=====================================================================================
  // Twist in the world frame
  //=====================================================================================

  struct Twist
  {
    double x;
    double y;
    double z;
    double yaw;

    constexpr Twist() : x{0}, y{0}, z{0}, yaw{0}
    {}

    void from_msg(const geometry_msgs::msg::Twist &msg)
    {
      x = msg.linear.x;
      y = msg.linear.y;
      z = msg.linear.z;
      yaw = msg.angular.z;
    }
  };

  std::ostream &operator<<(std::ostream &os, Twist const &t);

  //=====================================================================================
  // Twist in the body frame
  //=====================================================================================

  struct TwistBody
  {
    double forward;
    double strafe;
    double vertical;
    double yaw;

    constexpr TwistBody() : forward{0}, strafe{0}, vertical{0}, yaw{0}
    {}
  };

  //=====================================================================================
  // Acceleration in the world frame
  //=====================================================================================

  struct Acceleration
  {
    double x;
    double y;
    double z;
    double yaw;

    constexpr Acceleration() : x{0}, y{0}, z{0}, yaw{0}
    {}

    constexpr Acceleration(double _x, double _y, double _z, double _yaw) : x{_x}, y{_y}, z{_z}, yaw{_yaw}
    {}

    void add(const Acceleration &that)
    {
      x += that.x;
      y += that.y;
      z += that.z;
      yaw += that.yaw;
    }
  };

  std::ostream &operator<<(std::ostream &os, Acceleration const &a);

  //=====================================================================================
  // Acceleration in the body frame
  //=====================================================================================

  struct AccelerationBody
  {
    double forward;
    double strafe;
    double vertical;
    double yaw;

    constexpr AccelerationBody() : forward{0}, strafe{0}, vertical{0}, yaw{0}
    {}

    constexpr AccelerationBody(double _forward, double _strafe, double _vertical, double _yaw) :
      forward{_forward}, strafe{_strafe}, vertical{_vertical}, yaw{_yaw}
    {}
  };

  std::ostream &operator<<(std::ostream &os, AccelerationBody const &a);

  //=====================================================================================
  // Thruster efforts from joystick or pid controllers, in the body frame
  // Ranges from 1.0 for forward to -1.0 for reverse
  //=====================================================================================

  class Efforts
  {
    double forward_;
    double strafe_;
    double vertical_;
    double yaw_;

  public:

    Efforts() : forward_{0}, strafe_{0}, vertical_{0}, yaw_{0}
    {}

    double forward() const
    { return forward_; }

    double strafe() const
    { return strafe_; }

    double vertical() const
    { return vertical_; }

    double yaw() const
    { return yaw_; }

    void set_forward(double forward)
    { forward_ = clamp(forward, -1.0, 1.0); }

    void set_strafe(double strafe)
    { strafe_ = clamp(strafe, -1.0, 1.0); }

    void set_vertical(double vertical)
    { vertical_ = clamp(vertical, -1.0, 1.0); }

    void set_yaw(double yaw)
    { yaw_ = clamp(yaw, -1.0, 1.0); }

    void all_stop()
    {
      forward_ = 0;
      strafe_ = 0;
      vertical_ = 0;
      yaw_ = 0;
    }

    void from_acceleration(const double current_yaw, const Acceleration &u_bar)
    {
      // Convert from world frame to body frame
      double x_effort = Model::accel_to_effort_xy(u_bar.x);
      double y_effort = Model::accel_to_effort_xy(u_bar.y);
      double forward, strafe;
      rotate_frame(x_effort, y_effort, current_yaw, forward, strafe);

      set_forward(forward);
      set_strafe(strafe);
      set_vertical(Model::accel_to_effort_z(u_bar.z));
      set_yaw(Model::accel_to_effort_yaw(u_bar.yaw));
    }

    void to_acceleration(const double current_yaw, Acceleration &u_bar)
    {
      double forward_accel = Model::effort_to_accel_xy(forward_);
      double strafe_accel = Model::effort_to_accel_xy(strafe_);
      u_bar.z = Model::effort_to_accel_z(vertical_);
      u_bar.yaw = Model::effort_to_accel_yaw(yaw_);

      // Convert from body frame to world frame
      rotate_frame(forward_accel, strafe_accel, -current_yaw, u_bar.x, u_bar.y);
    }

    void scale(double factor)
    {
      // Scale efforts by a factor, useful for throttling
      set_forward(forward_ * factor);
      set_strafe(strafe_ * factor);
      set_vertical(vertical_ * factor);
      set_yaw(yaw_ * factor);
    }

    orca_msgs::msg::Efforts to_msg() const
    {
      orca_msgs::msg::Efforts msg;

      msg.forward = forward_;
      msg.strafe = strafe_;
      msg.vertical = vertical_;
      msg.yaw = yaw_;

      return msg;
    }

    void from_msg(const orca_msgs::msg::Efforts &msg)
    {
      forward_ = msg.forward;
      strafe_ = msg.strafe;
      vertical_ = msg.vertical;
      yaw_ = msg.yaw;

    }
  };

  std::ostream &operator<<(std::ostream &os, Efforts const &e);

  //=====================================================================================
  // Observation -- observation of a marker from a camera
  //=====================================================================================

  // Should be Observation::NOT_A_MARKER, but sometimes gives a linker error if < C++17
  constexpr int NOT_A_MARKER = -1;

  struct Observation
  {
    int id;                       // Marker ID
    cv::Point2d c0, c1, c2, c3;   // Corners
    double distance;              // Estimated distance to marker
    double yaw;                   // Yaw to center of marker

    Observation() : id{NOT_A_MARKER}
    {}

    void estimate_distance_and_yaw_from_corners(double marker_length, double hfov, double hres);

    void estimate_corners_from_distance_and_yaw(double marker_length, double hfov, double hres, double vres);

    void from_msg(const fiducial_vlam_msgs::msg::Observation &msg,
                  double marker_length, double hfov, double hres);
  };

  std::ostream &operator<<(std::ostream &os, Observation const &obs);

  //=====================================================================================
  // Fiducial pose (FP) -- pose and observations
  //=====================================================================================

  struct FP
  {
    PoseWithCovariance pose;
    std::vector<Observation> observations;

    // TODO naming is confusing -- fix

    // Get the closest observation and return the distance
    double closest_obs(Observation &obs) const;

    // Return the distance of the closest observation
    double closest_obs() const;

    // True if pose is good
    bool good_pose() const
    { return pose.good_pose() && closest_obs() < 1.8; }

    // True if there is at least one good observation
    bool good_obs() const
    { return !observations.empty(); }

    // True if there is a good observation of a particular marker
    bool good_obs(int id) const;

    // Get the observation of a particular marker, return true if successful
    bool get_obs(int id, Observation &obs) const;

    // True if z is good
    bool good_z() const
    { return pose.good_z(); }

    void from_msgs(
      const fiducial_vlam_msgs::msg::Observations &obs,
      const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg,
      double marker_length, double hfov, double hres);

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

    void from_msgs(
      const fiducial_vlam_msgs::msg::Observations &obs,
      const geometry_msgs::msg::PoseWithCovarianceStamped &fcam_msg,
      double marker_length, double hfov, double hres);

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

} // namespace orca_shared

#endif //ORCA_SHARED_GEOMETRY_HPP
