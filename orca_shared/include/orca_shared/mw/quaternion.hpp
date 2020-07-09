#ifndef ORCA_SHARED_MW_QUATERNION_HPP
#define ORCA_SHARED_MW_QUATERNION_HPP

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"
#include "orca_shared/util.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace mw
{

  class Quaternion
  {
    double roll_{};
    double pitch_{};
    double yaw_{};

  public:

    Quaternion() = default;

    explicit Quaternion(const geometry_msgs::msg::Quaternion &msg)
    {
      tf2::Quaternion q;
      tf2::fromMsg(msg, q);
      tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);
    }

    Quaternion(const double &roll, const double &pitch, const double &yaw) :
      roll_{roll},
      pitch_{pitch},
      yaw_{yaw}
    {}

    geometry_msgs::msg::Quaternion msg() const
    {
      tf2::Matrix3x3 m;
      m.setRPY(roll_, pitch_, yaw_);
      tf2::Quaternion q;
      m.getRotation(q);
      return tf2::toMsg(q);
    }

    double roll() const
    {
      return roll_;
    }

    double pitch() const
    {
      return pitch_;
    }

    double yaw() const
    {
      return yaw_;
    }

    void roll(const double &v)
    {
      roll_ = orca::norm_angle(v);
    }

    void pitch(const double &v)
    {
      pitch_ = orca::norm_angle(v);
    }

    void yaw(const double &v)
    {
      yaw_ = orca::norm_angle(v);
    }

    double distance_yaw(const Quaternion &that) const
    {
      return std::abs(orca::norm_angle(yaw() - that.yaw()));
    }

    double distance_yaw(const double &that) const
    {
      return std::abs(orca::norm_angle(yaw() - that));
    }

    Quaternion motion(const rclcpp::Duration &d, const mw::Twist &v0, const mw::Acceleration &a) const
    {
      auto dt = d.seconds();
      return Quaternion{roll_, pitch_, orca::norm_angle(yaw_ + v0.yaw() * dt + 0.5 * a.yaw() * dt * dt)};
    }

    Quaternion operator+(const Quaternion &that) const
    {
      return Quaternion{roll_ + that.roll_, pitch_ + that.pitch_, yaw_ + that.yaw_};
    }

    Quaternion operator-(const Quaternion &that) const
    {
      return Quaternion{roll_ - that.roll_, pitch_ - that.pitch_, yaw_ - that.yaw_};
    }

    Quaternion operator-() const
    {
      return Quaternion{-roll_, -pitch_, -yaw_};
    }

    bool operator==(const Quaternion &that) const
    {
      return roll_ == that.roll_ &&
             pitch_ == that.pitch_ &&
             yaw_ == that.yaw_;
    }

    bool operator!=(const Quaternion &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Quaternion &v);
  };

}

#endif //ORCA_SHARED_MW_QUATERNION_HPP
