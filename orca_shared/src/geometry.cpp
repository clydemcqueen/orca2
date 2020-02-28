#include "orca_shared/geometry.hpp"

#include <iomanip>

#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca
{

  //=====================================================================================
  // Pose in world frame
  //=====================================================================================

  geometry_msgs::msg::Pose Pose::to_msg() const
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

  void Pose::from_msg(const geometry_msgs::msg::Pose &msg)
  {
    // Convert ROS Pose to Orca2 Pose by dropping roll & pitch
    x = msg.position.x;
    y = msg.position.y;
    z = msg.position.z;
    yaw = get_yaw(msg.orientation);
  }

  // Distance between 2 poses on the xy plane
  double Pose::distance_xy(const Pose &that) const
  {
    return std::hypot(x - that.x, y - that.y);
  }

  double Pose::distance_xy(double _x, double _y) const
  {
    return std::hypot(x - _x, y - _y);
  }

  double Pose::distance_xy(const nav_msgs::msg::Odometry &msg) const
  {
    return std::hypot(x - msg.pose.pose.position.x, y - msg.pose.pose.position.y);
  }

  double Pose::distance_xy(const geometry_msgs::msg::Pose &msg) const
  {
    return std::hypot(x - msg.position.x, y - msg.position.y);
  }

  // Z distance between 2 poses
  double Pose::distance_z(const Pose &that) const
  {
    return std::abs(z - that.z);
  }

  double Pose::distance_z(const nav_msgs::msg::Odometry &msg) const
  {
    return std::abs(z - msg.pose.pose.position.z);
  }

  // Yaw distance between 2 poses
  double Pose::distance_yaw(const Pose &that) const
  {
    return std::abs(norm_angle(yaw - that.yaw));
  }

  double Pose::distance_yaw(const nav_msgs::msg::Odometry &msg) const
  {
    return std::abs(norm_angle(yaw - get_yaw(msg.pose.pose.orientation)));
  }

  std::ostream &operator<<(std::ostream &os, Pose const &pose)
  {
    return os << std::fixed << std::setprecision(2)
              << "{" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.yaw << "}";
  }

  //=====================================================================================
  // PoseStamped
  //=====================================================================================

  geometry_msgs::msg::PoseStamped PoseStamped::to_msg() const
  {
    geometry_msgs::msg::PoseStamped msg;

    msg.header.stamp = t;
    msg.pose = pose.to_msg();

    return msg;
  }

  void PoseStamped::from_msg(const geometry_msgs::msg::PoseStamped &msg)
  {
    t = msg.header.stamp;
    pose.from_msg(msg.pose);
  }

  void PoseStamped::from_msg(const nav_msgs::msg::Odometry &msg)
  {
    t = msg.header.stamp;
    pose.from_msg(msg.pose.pose);
  }

  void PoseStamped::from_msg(geometry_msgs::msg::PoseWithCovarianceStamped &msg)
  {
    t = msg.header.stamp;
    pose.from_msg(msg.pose.pose);
  }

  void PoseStamped::add_to_path(nav_msgs::msg::Path &path) const
  {
    auto msg = to_msg();
    msg.header.frame_id = path.header.frame_id;
    path.poses.push_back(msg);
  }

  //=====================================================================================
  // PoseWithCovariance
  //=====================================================================================

  void PoseWithCovariance::from_msg(const geometry_msgs::msg::PoseWithCovariance &pose_with_covariance)
  {
    pose.from_msg(pose_with_covariance.pose);
    x_valid = (pose_with_covariance.covariance[0 * 7] < 1e4);
    y_valid = (pose_with_covariance.covariance[1 * 7] < 1e4);
    z_valid = (pose_with_covariance.covariance[2 * 7] < 1e4);
    yaw_valid = (pose_with_covariance.covariance[5 * 7] < 1e4);
  }

  //=====================================================================================
  // Twist in the world frame
  //=====================================================================================

  void Twist::from_msg(const geometry_msgs::msg::Twist &msg)
  {
    x = msg.linear.x;
    y = msg.linear.y;
    z = msg.linear.z;
    yaw = msg.angular.z;
  }

  geometry_msgs::msg::Twist Twist::to_msg() const
  {
    geometry_msgs::msg::Twist msg;

    msg.linear.x = x;
    msg.linear.y = y;
    msg.linear.z = z;
    msg.angular.z = yaw;

    return msg;
  }

  std::ostream &operator<<(std::ostream &os, Twist const &t)
  {
    return os << std::fixed << std::setprecision(2)
              << "{" << t.x << ", " << t.y << ", " << t.z << ", " << t.yaw << "}";
  }

  //=====================================================================================
  // Acceleration in the world frame
  //=====================================================================================

  std::ostream &operator<<(std::ostream &os, Acceleration const &a)
  {
    return os << std::fixed << std::setprecision(2)
              << "{" << a.x << ", " << a.y << ", " << a.z << ", " << a.yaw << "}";
  }

  //=====================================================================================
  // Efforts
  //=====================================================================================

  void Efforts::set_forward(double forward)
  { forward_ = clamp(forward, -1.0, 1.0); }

  void Efforts::set_strafe(double strafe)
  { strafe_ = clamp(strafe, -1.0, 1.0); }

  void Efforts::set_vertical(double vertical)
  { vertical_ = clamp(vertical, -1.0, 1.0); }

  void Efforts::set_yaw(double yaw)
  { yaw_ = clamp(yaw, -1.0, 1.0); }

  void Efforts::all_stop()
  {
    forward_ = 0;
    strafe_ = 0;
    vertical_ = 0;
    yaw_ = 0;
  }

  void Efforts::from_acceleration(const orca::Model &model, const double current_yaw, const Acceleration &u_bar)
  {
    // Convert from world frame to body frame
    double x_effort = model.accel_to_effort_xy(u_bar.x);
    double y_effort = model.accel_to_effort_xy(u_bar.y);
    double forward, strafe;
    rotate_frame(x_effort, y_effort, current_yaw, forward, strafe);

    set_forward(forward);
    set_strafe(strafe);
    set_vertical(model.accel_to_effort_z(u_bar.z));
    set_yaw(model.accel_to_effort_yaw(u_bar.yaw));
  }

  void Efforts::to_acceleration(const orca::Model &model, const double current_yaw, Acceleration &u_bar)
  {
    double forward_accel = model.effort_to_accel_xy(forward_);
    double strafe_accel = model.effort_to_accel_xy(strafe_);
    u_bar.z = model.effort_to_accel_z(vertical_);
    u_bar.yaw = model.effort_to_accel_yaw(yaw_);

    // Convert from body frame to world frame
    rotate_frame(forward_accel, strafe_accel, -current_yaw, u_bar.x, u_bar.y);
  }

  void Efforts::scale(double factor)
  {
    // Scale efforts by a factor, useful for throttling
    set_forward(forward_ * factor);
    set_strafe(strafe_ * factor);
    set_vertical(vertical_ * factor);
    set_yaw(yaw_ * factor);
  }

  orca_msgs::msg::Efforts Efforts::to_msg() const
  {
    orca_msgs::msg::Efforts msg;

    msg.forward = forward_;
    msg.strafe = strafe_;
    msg.vertical = vertical_;
    msg.yaw = yaw_;

    return msg;
  }

  void Efforts::from_msg(const orca_msgs::msg::Efforts &msg)
  {
    forward_ = msg.forward;
    strafe_ = msg.strafe;
    vertical_ = msg.vertical;
    yaw_ = msg.yaw;
  }

  std::ostream &operator<<(std::ostream &os, Efforts const &e)
  {
    return os << std::fixed << std::setprecision(2)
              << "{" << e.forward() << ", " << e.strafe() << ", " << e.vertical() << ", " << e.yaw() << "}";
  }

} // namespace orca