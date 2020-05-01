#ifndef ORCA_SHARED_MW_TARGET_HPP
#define ORCA_SHARED_MW_TARGET_HPP

#include "orca_shared/mw/pose.hpp"

namespace mw
{

  class Target
  {
    int id_;
    Pose pose_;

  public:

    Target() = default;

    Target(const int &id, const Pose &pose) :
      id_{id},
      pose_{pose}
    {}

    const int &id() const
    {
      return id_;
    }

    const Pose &pose() const
    {
      return pose_;
    }

    int &id()
    {
      return id_;
    }

    Pose &pose()
    {
      return pose_;
    }

    bool operator==(const Target &that) const
    {
      return id_ == that.id_ &&
             pose_ == that.pose_;
    }

    bool operator!=(const Target &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, Target const &v);
  };

}

#endif //ORCA_SHARED_MW_TARGET_HPP
