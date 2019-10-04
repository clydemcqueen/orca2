#include "orca_base/util.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_base
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

  std::string to_str(const tf2::Transform &t)
  {
    double roll, pitch, yaw;
    t.getBasis().getRPY(roll, pitch, yaw);
    std::stringstream s;
    s <<
      "xyz(" << t.getOrigin().x() << ", " << t.getOrigin().y() << ", " << t.getOrigin().z() << ") " <<
      "rpy(" << roll << ", " << pitch << ", " << yaw << ") " <<
      "q(" << t.getRotation().x() << ", " << t.getRotation().y() << ", " << t.getRotation().z() << ", " <<
      t.getRotation().w() << ")";
    return s.str();
  }

  bool button_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int button)
  {
    return curr->buttons[button] && !prev.buttons[button];
  }

  bool trim_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int axis)
  {
    return curr->axes[axis] && !prev.axes[axis];
  }

} // namespace orca_base