#ifndef ORCA_SHARED_GEOMETRY_HPP
#define ORCA_SHARED_GEOMETRY_HPP

#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"

#include "orca_msgs/msg/efforts.hpp"
#include "orca_msgs/msg/pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// 4 DoF geometry and motion structs

namespace orca
{

  constexpr bool full_pose(const nav_msgs::msg::Odometry &odom)
  {
    return odom.pose.covariance[0] < 1e4;
  }

  //=====================================================================================
  // Pose
  //=====================================================================================

  struct Pose
  {
    double x;
    double y;
    double z;
    double yaw;

    constexpr Pose() : x{0}, y{0}, z{0}, yaw{0}
    {}

    void to_msg(geometry_msgs::msg::Pose &msg) const
    {
      msg.position.x = x;
      msg.position.y = y;
      msg.position.z = z;

      // Yaw to quaternion
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      msg.orientation = tf2::toMsg(q);
    }

    void to_msg(orca_msgs::msg::Pose &msg) const
    {
      msg.x = x;
      msg.y = y;
      msg.z = z;
      msg.yaw = yaw;
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

    Pose error(const Pose &that) const
    {
      Pose e;
      e.x = x - that.x;
      e.y = y - that.y;
      e.z = z - that.z;
      e.yaw = norm_angle(yaw - that.yaw);
      return e;
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

    void to_msg(geometry_msgs::msg::PoseStamped &msg) const
    {
      msg.header.stamp = t;
      pose.to_msg(msg.pose);
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
      geometry_msgs::msg::PoseStamped msg;
      to_msg(msg);
      msg.header.frame_id = path.header.frame_id;
      path.poses.push_back(msg);
    }
  };

  //=====================================================================================
  // Twist
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

  //=====================================================================================
  // Acceleration
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

    std::string str()
    {
      std::stringstream ss;
      ss << "(" << x << ", " << y << ", " << z << ", " << yaw << ")";
      return ss.str();
    }
  };

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

    void to_msg(orca_msgs::msg::Efforts &msg) const
    {
      msg.forward = forward_;
      msg.strafe = strafe_;
      msg.vertical = vertical_;
      msg.yaw = yaw_;
    }

    void from_msg(const orca_msgs::msg::Efforts &msg)
    {
      forward_ = msg.forward;
      strafe_ = msg.strafe;
      vertical_ = msg.vertical;
      yaw_ = msg.yaw;

    }
  };

} // namespace orca_shared

#endif //ORCA_SHARED_GEOMETRY_HPP
