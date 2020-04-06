#include "orca_filter/filter_base.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_filter
{

  //==================================================================
  // Measurement
  //==================================================================

  void Measurement::init_z(const orca_msgs::msg::Depth &depth,
                           const mw::Observations &observations, ukf::MeasurementFn h_fn)
  {
    type_ = Type::depth;
    stamp_ = depth.header.stamp;
    observations_ = observations;

    z_ = Eigen::VectorXd(1);
    z_ << depth.z;

    R_ = Eigen::MatrixXd(1, 1);
    R_ << depth.z_variance;

    h_fn_ = std::move(h_fn);

    // Use the standard residual and mean functions for depth measurements
    r_z_fn_ = ukf::residual;
    mean_z_fn_ = ukf::unscented_mean;
  }

  void Measurement::init_4dof(const geometry_msgs::msg::PoseWithCovarianceStamped &pose,
                              const mw::Observations &observations, ukf::MeasurementFn h_fn)
  {
    type_ = Type::four;
    stamp_ = pose.header.stamp;
    observations_ = observations;

    tf2::Transform t_map_base;
    tf2::fromMsg(pose.pose.pose, t_map_base);

    tf2Scalar roll, pitch, yaw;
    t_map_base.getBasis().getRPY(roll, pitch, yaw);

    z_ = Eigen::VectorXd(4);
    z_ << t_map_base.getOrigin().x(), t_map_base.getOrigin().y(), t_map_base.getOrigin().z(), yaw;

    R_ = Eigen::MatrixXd(4, 4);
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        // Copy rows {0, 1, 2, 5} and cols {0, 1, 2, 5}
        R_(i, j) = pose.pose.covariance[(i < 3 ? i : 5) * 6 + (j < 3 ? j : 5)];
      }
    }

    h_fn_ = std::move(h_fn);

    // Use a custom residual and mean functions for pose measurements
    r_z_fn_ = four_state_residual;
    mean_z_fn_ = four_state_mean;
  }

  void Measurement::init_6dof(const geometry_msgs::msg::PoseWithCovarianceStamped &pose,
                              const mw::Observations &observations, ukf::MeasurementFn h_fn)
  {
    type_ = Type::six;
    stamp_ = pose.header.stamp;
    observations_ = observations;

    tf2::Transform t_map_base;
    tf2::fromMsg(pose.pose.pose, t_map_base);

    tf2Scalar roll, pitch, yaw;
    t_map_base.getBasis().getRPY(roll, pitch, yaw);

    z_ = Eigen::VectorXd(6);
    z_ << t_map_base.getOrigin().x(), t_map_base.getOrigin().y(), t_map_base.getOrigin().z(), roll, pitch, yaw;

    R_ = Eigen::MatrixXd(6, 6);
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        R_(i, j) = pose.pose.covariance[i * 6 + j];
      }
    }

    h_fn_ = std::move(h_fn);

    // Use a custom residual and mean functions for pose measurements
    r_z_fn_ = six_state_residual;
    mean_z_fn_ = six_state_mean;
  }

  std::string Measurement::name()
  {
    if (type_ == Type::depth) {
      return "z";
    } else if (type_ == Type::four) {
      return "4dof";
    } else {
      return "6dof";
    }
  }

} // namespace orca_base