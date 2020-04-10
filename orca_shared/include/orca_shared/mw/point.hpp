#ifndef ORCA_SHARED_MW_POINT_HPP
#define ORCA_SHARED_MW_POINT_HPP

#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "orca_shared/mw/acceleration.hpp"
#include "orca_shared/mw/twist.hpp"

namespace mw
{

  class Point
  {
    geometry_msgs::msg::Point msg_;

  public:

    Point() = default;

    explicit Point(const geometry_msgs::msg::Point &msg) :
      msg_{msg}
    {}

    Point(const double &x, const double &y, const double &z)
    {
      msg_.x = x;
      msg_.y = y;
      msg_.z = z;
    }

    geometry_msgs::msg::Point msg() const
    {
      return msg_;
    }

    double x() const
    {
      return msg_.x;
    }

    double y() const
    {
      return msg_.y;
    }

    double z() const
    {
      return msg_.z;
    }

    double &x()
    {
      return msg_.x;
    }

    double &y()
    {
      return msg_.y;
    }

    double &z()
    {
      return msg_.z;
    }

    void x(const double &v)
    {
      msg_.x = v;
    }

    void y(const double &v)
    {
      msg_.y = v;
    }

    void z(const double &v)
    {
      msg_.z = v;
    }

    double distance_xy(const Point &that) const
    {
      return std::hypot(x() - that.x(), y() - that.y());
    }

    double distance_xy(const double &_x, const double &_y) const
    {
      return std::hypot(x() - _x, y() - _y);
    }

    double distance_z(const Point &that) const
    {
      return std::abs(z() - that.z());
    }

    Point move(const rclcpp::Duration &d, const mw::Twist &v0, const mw::Acceleration &a) const
    {
      auto dt = d.seconds();
      return Point{x() + v0.x() * dt + 0.5 * a.x() * dt * dt,
                   y() + v0.y() * dt + 0.5 * a.y() * dt * dt,
                   z() + v0.z() * dt + 0.5 * a.z() * dt * dt};
    }

    Point operator+(const Point &that) const
    {
      return Point{x() + that.x(), y() + that.y(), z() + that.z()};
    }

    Point operator-(const Point &that) const
    {
      return Point{x() - that.x(), y() - that.y(), z() - that.z()};
    }

    Point operator-() const
    {
      return Point{-x(), -y(), -z()};
    }

    bool operator==(const Point &that) const
    {
      return msg_ == that.msg_;
    }

    bool operator!=(const Point &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Point &v);
  };

}

#endif //ORCA_SHARED_MW_POINT_HPP
