#ifndef ORCA_SHARED_MW_POSE_WITH_COVARIANCE_HPP
#define ORCA_SHARED_MW_POSE_WITH_COVARIANCE_HPP

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "orca_shared/mw/pose.hpp"

namespace mw
{

  using CovarianceType = std::array<double, 36>;

  class PoseWithCovariance
  {
    Pose pose_;
    CovarianceType covariance_{};

  public:

    constexpr static int X_IX = 0 * 7;
    constexpr static int Y_IX = 1 * 7;
    constexpr static int Z_IX = 2 * 7;
    constexpr static int ROLL_IX = 3 * 7;
    constexpr static int PITCH_IX = 4 * 7;
    constexpr static int YAW_IX = 5 * 7;

    constexpr static double THRESHOLD = 1e4;
    constexpr static double VERY_HIGH = 1e5;

    constexpr static int DOF_NONE = 0;
    constexpr static int DOF_Z = 1;
    constexpr static int DOF_FOUR = 2;
    constexpr static int DOF_SIX = 3;

    PoseWithCovariance() = default;

    explicit PoseWithCovariance(const geometry_msgs::msg::PoseWithCovariance &msg) :
      pose_{msg.pose},
      covariance_{msg.covariance}
    {}

    PoseWithCovariance(const Pose &pose, const CovarianceType &covariance, int dof = DOF_SIX) :
      pose_{pose},
      covariance_{covariance}
    {
      // Slam the covariance as required
      if (dof != DOF_SIX) {
        covariance_[ROLL_IX] = VERY_HIGH;
        covariance_[PITCH_IX] = VERY_HIGH;
      }
      if (dof != DOF_SIX && dof != DOF_FOUR) {
        covariance_[X_IX] = VERY_HIGH;
        covariance_[Y_IX] = VERY_HIGH;
        covariance_[YAW_IX] = VERY_HIGH;
      }
      if (dof != DOF_SIX && dof != DOF_FOUR && dof != DOF_Z) {
        covariance_[Z_IX] = VERY_HIGH;
      }
    }

    geometry_msgs::msg::PoseWithCovariance msg() const
    {
      geometry_msgs::msg::PoseWithCovariance msg;
      msg.pose = pose_.msg();
      msg.covariance = covariance_;
      return msg;
    }

    const Pose &pose() const
    {
      return pose_;
    }

    const std::array<double, 36> &covariance() const
    {
      return covariance_;
    }

    Pose &pose()
    {
      return pose_;
    }

    std::array<double, 36> &covariance()
    {
      return covariance_;
    }

    int dof() const
    {
      if (covariance_[Z_IX] > THRESHOLD) {
        return DOF_NONE;
      } else if (covariance_[X_IX] > THRESHOLD || covariance_[Y_IX] > THRESHOLD || covariance_[YAW_IX] > THRESHOLD) {
        return DOF_Z;
      } else if (covariance_[ROLL_IX] > THRESHOLD || covariance_[PITCH_IX] > THRESHOLD) {
        return DOF_FOUR;
      } else {
        return DOF_SIX;
      }
    }

    bool good1() const
    {
      auto d = dof();
      return d == DOF_SIX || d == DOF_FOUR || d == DOF_Z;
    }

    bool good4() const
    {
      auto d = dof();
      return d == DOF_SIX || d == DOF_FOUR;
    }

    bool good6() const
    {
      return dof() == DOF_SIX;
    }

    bool operator==(const PoseWithCovariance &that) const
    {
      return pose_ == that.pose_ &&
             covariance_ == that.covariance_;
    }

    bool operator!=(const PoseWithCovariance &that) const
    {
      return !(*this == that);
    }


    friend std::ostream &operator<<(std::ostream &os, const PoseWithCovariance &v);
  };

}

#endif //ORCA_SHARED_MW_POSE_WITH_COVARIANCE_HPP
