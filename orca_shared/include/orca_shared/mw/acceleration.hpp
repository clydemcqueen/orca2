#ifndef ORCA_SHARED_MW_ACCELERATION_HPP
#define ORCA_SHARED_MW_ACCELERATION_HPP

#include <ostream>

namespace mw
{

  class Acceleration
  {
    double x_;
    double y_;
    double z_;
    double yaw_;

  public:

    Acceleration() = default;

    Acceleration(double x, double y, double z, double yaw) :
      x_{x},
      y_{y},
      z_{z},
      yaw_{yaw}
    {}

    double x() const
    {
      return x_;
    }

    double y() const
    {
      return y_;
    }

    double z() const
    {
      return z_;
    }

    double yaw() const
    {
      return yaw_;
    }

    double &x()
    {
      return x_;
    }

    double &y()
    {
      return y_;
    }

    double &z()
    {
      return z_;
    }

    double &yaw()
    {
      return yaw_;
    }

    void x(const double &v)
    {
      x_ = v;
    }

    void y(const double &v)
    {
      y_ = v;
    }

    void z(const double &v)
    {
      z_ = v;
    }

    void yaw(const double &v)
    {
      yaw_ = v;
    }

    Acceleration operator+(const Acceleration &that) const
    {
      return Acceleration{x_ + that.x_, y_ + that.y_, z_ + that.z_, yaw_ + that.yaw_};
    }

    Acceleration operator-(const Acceleration &that) const
    {
      return Acceleration{x_ - that.x_, y_ - that.y_, z_ - that.z_, yaw_ - that.yaw_};
    }

    Acceleration operator-() const
    {
      return Acceleration{-x_, -y_, -z_, -yaw_};
    }

    bool operator==(const Acceleration &that) const
    {
      return x_ == that.x_ &&
             y_ == that.y_ &&
             z_ == that.z_ &&
             yaw_ == that.yaw_;
    }

    bool operator!=(const Acceleration &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Acceleration &v);
  };

}

#endif //ORCA_SHARED_MW_ACCELERATION_HPP
