#ifndef ORCA_SHARED_MW_TWIST_BODY_HPP
#define ORCA_SHARED_MW_TWIST_BODY_HPP

#include <ostream>

namespace mw
{

  class TwistBody
  {
    double forward_{};
    double strafe_{};
    double vertical_{};
    double yaw_{};

  public:

    TwistBody() = default;

    TwistBody(double forward, double strafe, double vertical, double yaw) :
      forward_{forward},
      strafe_{strafe},
      vertical_{vertical},
      yaw_{yaw}
    {}

    double forward() const
    {
      return forward_;
    }

    double strafe() const
    {
      return strafe_;
    }

    double vertical() const
    {
      return vertical_;
    }

    double yaw() const
    {
      return yaw_;
    }

    double &forward()
    {
      return forward_;
    }

    double &strafe()
    {
      return strafe_;
    }

    double &vertical()
    {
      return vertical_;
    }

    double &yaw()
    {
      return yaw_;
    }

    void forward(const double &v)
    {
      forward_ = v;
    }

    void strafe(const double &v)
    {
      strafe_ = v;
    }

    void vertical(const double &v)
    {
      vertical_ = v;
    }

    void yaw(const double &v)
    {
      yaw_ = v;
    }

    TwistBody operator+(const TwistBody &that) const
    {
      return TwistBody{forward_ + that.forward_, strafe_ + that.strafe_,
                              vertical_ + that.vertical_, yaw_ + that.yaw_};
    }

    TwistBody operator-(const TwistBody &that) const
    {
      return TwistBody{forward_ - that.forward_, strafe_ - that.strafe_,
                              vertical_ - that.vertical_, yaw_ - that.yaw_};
    }

    TwistBody operator-() const
    {
      return TwistBody{-forward_, -strafe_, -vertical_, -yaw_};
    }

    bool operator==(const TwistBody &that) const
    {
      return forward_ == that.forward_ &&
             strafe_ == that.strafe_ &&
             vertical_ == that.vertical_ &&
             yaw_ == that.yaw_;
    }

    bool operator!=(const TwistBody &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const TwistBody &v);
  };

}

#endif //ORCA_SHARED_MW_TWIST_BODY_HPP
