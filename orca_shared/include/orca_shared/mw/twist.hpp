#ifndef ORCA_SHARED_MW_TWIST_HPP
#define ORCA_SHARED_MW_TWIST_HPP

#include "geometry_msgs/msg/twist.hpp"

namespace mw
{

  class Twist
  {
    geometry_msgs::msg::Twist msg_;

  public:

    Twist() = default;

    explicit Twist(const geometry_msgs::msg::Twist &msg) :
      msg_{msg}
    {}

    Twist(const double &x, const double &y, const double &z, const double &yaw)
    {
      msg_.linear.x = x;
      msg_.linear.y = y;
      msg_.linear.z = z;
      msg_.angular.z = yaw;
    }

    geometry_msgs::msg::Twist msg() const
    {
      return msg_;
    }

    double x() const
    {
      return msg_.linear.x;
    }

    double y() const
    {
      return msg_.linear.y;
    }

    double z() const
    {
      return msg_.linear.z;
    }

    double yaw() const
    {
      return msg_.angular.z;
    }

    double &x()
    {
      return msg_.linear.x;
    }

    double &y()
    {
      return msg_.linear.y;
    }

    double &z()
    {
      return msg_.linear.z;
    }

    double &yaw()
    {
      return msg_.angular.z;
    }

    void x(const double &v)
    {
      msg_.linear.x = v;
    }

    void y(const double &v)
    {
      msg_.linear.y = v;
    }

    void z(const double &v)
    {
      msg_.linear.z = v;
    }

    void yaw(const double &v)
    {
      msg_.angular.z = v;
    }

    Twist move(const rclcpp::Duration &d, const mw::Acceleration &a) const
    {
      double dt = d.seconds();
      return Twist{x() + a.x() * dt,
                   y() + a.y() * dt,
                   z() + a.z() * dt,
                   yaw() + a.yaw() * dt};
    }

    bool operator==(const Twist &that) const
    {
      return msg_ == that.msg_;
    }

    bool operator!=(const Twist &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Twist &v);
  };

}

#endif //ORCA_SHARED_MW_TWIST_HPP
