#ifndef ORCA_SHARED_MW_POSE_WITH_COVARIANCE_HPP
#define ORCA_SHARED_MW_POSE_WITH_COVARIANCE_HPP

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "orca_shared/mw/pose.hpp"

namespace mw
{

  enum class CovarianceDoF
  {
    none,   // Covariance is not valid
    one,    // Covariance valid for z
    four,   // Covariance valid for x, y, z, yaw
    six,    // Covariance valid for x, y, z, roll, pitch, yaw
  };

  using CovarianceType = std::array<double, 36>;

  class PoseWithCovariance
  {
    Pose pose_;
    CovarianceDoF covariance_dof_{CovarianceDoF::none};
    CovarianceType covariance_;

    constexpr static int X_IX = 0 * 7;
    constexpr static int Y_IX = 1 * 7;
    constexpr static int Z_IX = 2 * 7;
    constexpr static int ROLL_IX = 3 * 7;
    constexpr static int PITCH_IX = 4 * 7;
    constexpr static int YAW_IX = 5 * 7;
    constexpr static double THRESHOLD = 1e4;
    constexpr static double VERY_HIGH = 1e5;

  public:

    PoseWithCovariance() = default;

    explicit PoseWithCovariance(const geometry_msgs::msg::PoseWithCovariance &msg) :
      pose_{msg.pose},
      covariance_{msg.covariance}
    {
      // Hack: use a high covariance value to set covariance_dof_
      if (covariance_[Z_IX] > THRESHOLD) {
        covariance_dof_ = CovarianceDoF::none;
      } else if (covariance_[X_IX] > THRESHOLD || covariance_[Y_IX] > THRESHOLD || covariance_[YAW_IX] > THRESHOLD) {
        covariance_dof_ = CovarianceDoF::one;
      } else if (covariance_[ROLL_IX] > THRESHOLD || covariance_[PITCH_IX] > THRESHOLD) {
        covariance_dof_ = CovarianceDoF::four;
      } else {
        covariance_dof_ = CovarianceDoF::six;
      }
    }

    PoseWithCovariance(const Pose &pose, const CovarianceDoF &covariance_dof, const CovarianceType &covariance) :
      pose_{pose},
      covariance_dof_{covariance_dof},
      covariance_{covariance}
    {}

    geometry_msgs::msg::PoseWithCovariance msg() const
    {
      geometry_msgs::msg::PoseWithCovariance msg;
      msg.pose = pose_.msg();
      msg.covariance = covariance_;
      if (!good6()) {
        msg.covariance[ROLL_IX] = VERY_HIGH;
        msg.covariance[PITCH_IX] = VERY_HIGH;
      }
      if (!good4()) {
        msg.covariance[X_IX] = VERY_HIGH;
        msg.covariance[Y_IX] = VERY_HIGH;
        msg.covariance[YAW_IX] = VERY_HIGH;
      }
      if (!good1()) {
        msg.covariance[Z_IX] = VERY_HIGH;
      }
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

    bool good1() const
    {
      return covariance_dof_ == CovarianceDoF::six || covariance_dof_ == CovarianceDoF::four ||
             covariance_dof_ == CovarianceDoF::one;
    }

    bool good4() const
    {
      return covariance_dof_ == CovarianceDoF::six || covariance_dof_ == CovarianceDoF::four;
    }

    bool good6() const
    {
      return covariance_dof_ == CovarianceDoF::six;
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
