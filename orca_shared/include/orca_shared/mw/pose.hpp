#ifndef ORCA_SHARED_MW_POSE_HPP
#define ORCA_SHARED_MW_POSE_HPP

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "orca_shared/mw/point.hpp"
#include "orca_shared/mw/quaternion.hpp"

namespace mw
{

  class Pose
  {
    Point position_;
    Quaternion orientation_;

  public:

    Pose() = default;

    explicit Pose(const geometry_msgs::msg::Pose &msg) :
      position_{msg.position},
      orientation_{msg.orientation}
    {}

    Pose(const Point &position, const Quaternion &orientation) :
      position_{position},
      orientation_{orientation}
    {}

    Pose(const double &x, const double &y, const double &z, const double &yaw) :
      position_{x, y, z},
      orientation_{0, 0, yaw}
    {}

    geometry_msgs::msg::Pose msg() const
    {
      geometry_msgs::msg::Pose msg;
      msg.position = position_.msg();
      msg.orientation = orientation_.msg();
      return msg;
    }

    const Point &position() const
    {
      return position_;
    }

    const Quaternion &orientation() const
    {
      return orientation_;
    }

    Point &position()
    {
      return position_;
    }

    Quaternion &orientation()
    {
      return orientation_;
    }

    tf2::Transform transform() const
    {
      tf2::Transform transform;
      tf2::fromMsg(msg(), transform);
      return transform;
    }

    // Project pose at time t to time t+d, given initial velocity v0 and acceleration a
    // Pose project(const rclcpp::Duration &d, const Twist &v0, const Acceleration &a);

    bool operator==(const Pose &that) const
    {
      return position_ == that.position_ &&
             orientation_ == that.orientation_;
    }

    bool operator!=(const Pose &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Pose &v);
  };

}

#endif //ORCA_SHARED_MW_POSE_HPP
