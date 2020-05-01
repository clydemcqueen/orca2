#ifndef ORCA_SHARED_MW_POSE_STAMPED_HPP
#define ORCA_SHARED_MW_POSE_STAMPED_HPP

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "orca_shared/mw/header.hpp"
#include "orca_shared/mw/pose.hpp"

namespace mw
{

  class PoseStamped
  {
    Header header_;
    Pose pose_;

  public:

    PoseStamped() = default;

    explicit PoseStamped(const geometry_msgs::msg::PoseStamped &msg) :
      header_{msg.header},
      pose_{msg.pose}
    {}

    PoseStamped(const Header &header, const Pose &pose) :
      header_{header},
      pose_{pose}
    {}

    geometry_msgs::msg::PoseStamped msg() const
    {
      geometry_msgs::msg::PoseStamped msg;
      msg.header = header_.msg();
      msg.pose = pose_.msg();
      return msg;
    }

    const Header &header() const
    {
      return header_;
    }

    const Pose &pose() const
    {
      return pose_;
    }

    Header &header()
    {
      return header_;
    }

    Pose &pose()
    {
      return pose_;
    }

    bool operator==(const PoseStamped &that) const
    {
      return header_ == that.header_ &&
             pose_ == that.pose_;
    }

    bool operator!=(const PoseStamped &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, PoseStamped const &v);
  };

}

#endif //ORCA_SHARED_MW_POSE_STAMPED_HPP
