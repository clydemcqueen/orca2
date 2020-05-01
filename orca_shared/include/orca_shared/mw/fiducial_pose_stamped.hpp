#ifndef ORCA_SHARED_MW_FIDUCIAL_POSE_STAMPED_HPP
#define ORCA_SHARED_MW_FIDUCIAL_POSE_STAMPED_HPP

#include <nav_msgs/msg/path__struct.hpp>
#include "orca_msgs/msg/fiducial_pose_stamped.hpp"
#include "orca_shared/mw/fiducial_pose.hpp"
#include "orca_shared/mw/header.hpp"
#include "orca_shared/mw/pose_stamped.hpp"

namespace mw
{

  class FiducialPoseStamped
  {
    Header header_;
    FiducialPose fp_;

  public:

    FiducialPoseStamped() = default;

    explicit FiducialPoseStamped(const orca_msgs::msg::FiducialPoseStamped &msg) :
      header_{msg.header},
      fp_{msg.fp}
    {}

    // From fiducial_vlam_msgs + geometry_msgs
    FiducialPoseStamped(const double &marker_length,
                        const geometry_msgs::msg::Pose &cam_f_base,
                        const fiducial_vlam_msgs::msg::Observations &vlam_observations,
                        const geometry_msgs::msg::PoseWithCovariance &pose) :
      header_{vlam_observations.header},
      fp_{marker_length, cam_f_base, vlam_observations, pose}
    {}

    FiducialPoseStamped(const Header &header, const FiducialPose &fp) :
      header_{header},
      fp_{fp}
    {}

    explicit FiducialPoseStamped(const Observer &observer) :
      fp_{observer}
    {}

    orca_msgs::msg::FiducialPoseStamped msg() const
    {
      orca_msgs::msg::FiducialPoseStamped msg;
      msg.header = header_.msg();
      msg.fp = fp_.msg();
      return msg;
    }

    const Header &header() const
    {
      return header_;
    }

    const FiducialPose &fp() const
    {
      return fp_;
    }

    Header &header()
    {
      return header_;
    }

    FiducialPose &fp()
    {
      return fp_;
    }

    void add_to_path(nav_msgs::msg::Path &path)
    {
      path.poses.push_back(PoseStamped{header(), fp().pose().pose()}.msg());
    }

    bool operator==(const FiducialPoseStamped &that) const
    {
      return header_ == that.header_ &&
             fp_ == that.fp_;
    }

    bool operator!=(const FiducialPoseStamped &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const FiducialPoseStamped &v);
  };

}

#endif //ORCA_SHARED_MW_FIDUCIAL_POSE_STAMPED_HPP
