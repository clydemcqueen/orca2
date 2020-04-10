#include "orca_shared/util.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca
{

  void rotate_frame(const double x, const double y, const double theta, double &x_r, double &y_r)
  {
    x_r = x * cos(theta) + y * sin(theta);
    y_r = y * cos(theta) - x * sin(theta);
  }

  void get_rpy(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw)
  {
    tf2::Quaternion tf2_q;
    tf2::fromMsg(q, tf2_q);
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
  }

  double get_yaw(const geometry_msgs::msg::Quaternion &q)
  {
    double roll = 0, pitch = 0, yaw = 0;
    get_rpy(q, roll, pitch, yaw);
    return yaw;
  }

  std::string to_str_rpy(const tf2::Transform &t)
  {
    double roll, pitch, yaw;
    t.getBasis().getRPY(roll, pitch, yaw);
    std::stringstream s;
    s <<
      "xyz(" << t.getOrigin().x() << ", " << t.getOrigin().y() << ", " << t.getOrigin().z() << ") " <<
      "rpy(" << roll << ", " << pitch << ", " << yaw << ") ";
    return s.str();
  }

  std::string to_str_q(const tf2::Transform &t)
  {
    std::stringstream s;
    s <<
      "xyz(" << t.getOrigin().x() << ", " << t.getOrigin().y() << ", " << t.getOrigin().z() << ") " <<
      "q(" << t.getRotation().x() << ", " << t.getRotation().y() << ", " << t.getRotation().z() << ", " <<
      t.getRotation().w() << ")";
    return s.str();
  }

  std::string to_str(const rclcpp::Time &t)
  {
    return to_str(builtin_interfaces::msg::Time{t});
  }

  std::string to_str(const builtin_interfaces::msg::Time &t)
  {
    std::stringstream s;
    s << "{" << t.sec << "s + " << t.nanosec << "ns (~" << static_cast<int>(t.nanosec / 1000000) << "ms)}";
    return s.str();
  }

  bool valid_stamp(const rclcpp::Time &stamp)
  {
    return stamp.nanoseconds() > 0;
  }

  tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose &pose)
  {
    tf2::Transform result;
    tf2::fromMsg(pose, result);
    return result;
  }

  geometry_msgs::msg::Pose transform_to_pose(const tf2::Transform &transform)
  {
    geometry_msgs::msg::Pose result;
    tf2::toMsg(transform, result);
    return result;
  }

} // namespace orca