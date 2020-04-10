#ifndef ORCA_SHARED_MW_POSE_HPP
#define ORCA_SHARED_MW_POSE_HPP

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "orca_shared/mw/acceleration.hpp"
#include "orca_shared/mw/point.hpp"
#include "orca_shared/mw/quaternion.hpp"
#include "orca_shared/mw/twist.hpp"

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

    double x() const
    {
      return position_.x();
    }

    double y() const
    {
      return position_.y();
    }

    double z() const
    {
      return position_.z();
    }

    double &x()
    {
      return position_.x();
    }

    double &y()
    {
      return position_.y();
    }

    double &z()
    {
      return position_.z();
    }

    double roll() const
    {
      return orientation_.roll();
    }

    double pitch() const
    {
      return orientation_.pitch();
    }

    double yaw() const
    {
      return orientation_.yaw();
    }

    void roll(const double &v)
    {
      orientation_.roll(v);
    }

    void pitch(const double &v)
    {
      orientation_.pitch(v);
    }

    void yaw(const double &v)
    {
      orientation_.yaw(v);
    }

    // TODO return ref to avoid possible assignment errors, e.g., pose_.transform() = my_transform;
    tf2::Transform transform() const
    {
      tf2::Transform transform;
      tf2::fromMsg(msg(), transform);
      return transform;
    }

    Pose move(const rclcpp::Duration &d, const mw::Twist &v0, const mw::Acceleration &a) const
    {
      return Pose{position_.move(d, v0, a), orientation_.move(d, v0, a)};
    }

    Pose operator+(const Pose &that) const
    {
      return Pose{position_ + that.position_, orientation_ + that.orientation_};
    }

    Pose operator-(const Pose &that) const
    {
      return Pose{position_ - that.position_, orientation_ - that.orientation_};
    }

    Pose operator-() const
    {
      return Pose{-position_, -orientation_};
    }

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
