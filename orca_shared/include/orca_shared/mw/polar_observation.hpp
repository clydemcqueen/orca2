#ifndef ORCA_SHARED_MW_POLAR_HPP
#define ORCA_SHARED_MW_POLAR_HPP

#include "orca_msgs/msg/polar_observation.hpp"

namespace mw
{

  class PolarObservation
  {
    orca_msgs::msg::PolarObservation msg_;

  public:

    static const PolarObservation None;

    PolarObservation() = default;

    explicit PolarObservation(const orca_msgs::msg::PolarObservation &msg) :
      msg_{msg}
    {}

    PolarObservation(const int &id, const double &distance, const double &bearing)
    {
      msg_.id = id;
      msg_.distance = distance;
      msg_.bearing = bearing;
    }

    orca_msgs::msg::PolarObservation msg() const
    {
      return msg_;
    }

    int id() const
    {
      return msg_.id;
    }

    double distance() const
    {
      return msg_.distance;
    }

    double bearing() const
    {
      return msg_.bearing;
    }

    int &id()
    {
      return msg_.id;
    }

    double &distance()
    {
      return msg_.distance;
    }

    double &bearing()
    {
      return msg_.bearing;
    }

    void id(const int &v)
    {
      msg_.id = v;
    }

    void distance(const double &v)
    {
      msg_.distance = v;
    }

    void bearing(const double &v)
    {
      msg_.bearing = v;
    }

    bool operator==(const PolarObservation &that) const
    {
      return msg_ == that.msg_;
    }

    bool operator!=(const PolarObservation &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, PolarObservation const &v);
  };

}

#endif //ORCA_SHARED_MW_POLAR_HPP
