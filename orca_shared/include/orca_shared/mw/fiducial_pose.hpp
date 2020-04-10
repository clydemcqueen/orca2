#ifndef ORCA_SHARED_MW_FIDUCIAL_POSE_HPP
#define ORCA_SHARED_MW_FIDUCIAL_POSE_HPP

#include "geometry_msgs/msg/pose.hpp"
#include "orca_msgs/msg/fiducial_pose.hpp"
#include "orca_shared/mw/map.hpp"
#include "orca_shared/mw/observations.hpp"
#include "orca_shared/mw/pose_with_covariance.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace mw
{

  class FiducialPose
  {
    Observations observations_;
    PoseWithCovariance pose_;

  public:

    FiducialPose() = default;

    explicit FiducialPose(const orca_msgs::msg::FiducialPose &msg) :
      observations_{msg.observations},
      pose_{msg.pose}
    {}

    // From fiducial_vlam_msgs + geometry_msgs
    FiducialPose(const double &marker_length,
                 const geometry_msgs::msg::Pose &cam_f_base,
                 const fiducial_vlam_msgs::msg::Observations &vlam_observations,
                 const geometry_msgs::msg::PoseWithCovariance &pose) :
      observations_{marker_length, cam_f_base, vlam_observations},
      pose_{pose}
    {}

    FiducialPose(const Observations &observations, const PoseWithCovariance &pose) :
      observations_{observations},
      pose_{pose}
    {}

    FiducialPose(const Observer &observer) :
      observations_{observer}
    {}

    orca_msgs::msg::FiducialPose msg() const
    {
      orca_msgs::msg::FiducialPose msg;
      msg.observations = observations_.msg();
      msg.pose = pose_.msg();
      return msg;
    }

    const Observations &observations() const
    {
      return observations_;
    }

    const PoseWithCovariance &pose() const
    {
      return pose_;
    }

    Observations &observations()
    {
      return observations_;
    }

    PoseWithCovariance &pose()
    {
      return pose_;
    }

    bool good(const double &max_distance) const
    {
      return pose().good4() && observations().closest_distance() < max_distance;
    }

    /**
     * Clear observations, and predict observations from a map
     * @param map Map of markers
     * @return Number of predicted observations
     */
    int predict_observations(const Map &map);

    bool operator==(const FiducialPose &that) const
    {
      return observations_ == that.observations_ &&
             pose_ == that.pose_;
    }

    bool operator!=(const FiducialPose &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const FiducialPose &v);
  };
}

#endif //ORCA_SHARED_MW_FIDUCIAL_POSE_HPP
