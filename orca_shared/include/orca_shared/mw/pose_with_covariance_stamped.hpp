#ifndef ORCA_SHARED_MW_POSE_WITH_COVARIANCE_STAMPED_HPP
#define ORCA_SHARED_MW_POSE_WITH_COVARIANCE_STAMPED_HPP

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "orca_shared/mw/header.hpp"
#include "orca_shared/mw/pose_with_covariance.hpp"

namespace mw
{

  class PoseWithCovarianceStamped
  {
    Header header_;
    PoseWithCovariance pose_;

  public:

    PoseWithCovarianceStamped() = default;

    explicit PoseWithCovarianceStamped(const geometry_msgs::msg::PoseWithCovarianceStamped &msg) :
      header_{msg.header},
      pose_{msg.pose}
    {}

    PoseWithCovarianceStamped(const Header &header, const PoseWithCovariance &pose) :
      header_{header},
      pose_{pose}
    {}

    geometry_msgs::msg::PoseWithCovarianceStamped msg() const
    {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header = header_.msg();
      msg.pose = pose_.msg();
      return msg;
    }

    const Header &header() const
    {
      return header_;
    }

    const PoseWithCovariance &pose() const
    {
      return pose_;
    }

    Header &header()
    {
      return header_;
    }

    PoseWithCovariance &pose()
    {
      return pose_;
    }

    bool operator==(const PoseWithCovarianceStamped &that) const
    {
      return header_ == that.header_ &&
             pose_ == that.pose_;
    }

    bool operator!=(const PoseWithCovarianceStamped &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, PoseWithCovarianceStamped const &v);
  };

}

#endif //ORCA_SHARED_MW_POSE_WITH_COVARIANCE_STAMPED_HPP
