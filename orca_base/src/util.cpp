#include "orca_base/util.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_base
{

  void rotate_frame(const double x, const double y, const double theta, double &x_r, double &y_r)
  {
    x_r = x * cos(theta) + y * sin(theta);
    y_r = y * cos(theta) - x * sin(theta);
  }

  double get_yaw(const geometry_msgs::msg::Quaternion &geometry_q)
  {
    tf2::Quaternion tf2_q;
    tf2::fromMsg(geometry_q, tf2_q);
    double roll = 0, pitch = 0, yaw = 0;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    return yaw;
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