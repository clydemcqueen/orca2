#include "orca_base/filter.hpp"

#include "eigen3/Eigen/Dense"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_base
{
  using namespace Eigen;

  constexpr int STATE_DIM = 12;       // [x, vx, y, vy, z, vz, roll, vroll, pitch, vpitch, yaw, vyaw]T
  constexpr int MEASUREMENT_DIM = 6;  // [x, y, z, roll, pitch, yaw]T
  constexpr int CONTROL_DIM = 4;      // [ax, ay, az, ayaw]T

  //==================================================================
  // Utility functions
  //==================================================================

  // Create measurement matrix z
  void to_z(const tf2::Transform &in, Eigen::MatrixXd &out)
  {
    const tf2::Vector3 &z_p = in.getOrigin();
    const tf2::Matrix3x3 &z_r = in.getBasis();

    tf2Scalar roll, pitch, yaw;
    z_r.getRPY(roll, pitch, yaw);

    out = Eigen::MatrixXd(MEASUREMENT_DIM, 1);
    out << z_p.x(), z_p.y(), z_p.z(), roll, pitch, yaw;
  }

  // Create measurement covariance matrix R
  void to_R(const std::array<double, 36> &in, Eigen::MatrixXd &out)
  {
    out = Eigen::MatrixXd(MEASUREMENT_DIM, MEASUREMENT_DIM);
    for (int i = 0; i < MEASUREMENT_DIM; i++) {
      for (int j = 0; j < MEASUREMENT_DIM; j++) {
        out(i, j) = in[i * MEASUREMENT_DIM + j];
      }
    }
  }

  //==================================================================
  // Filter
  //==================================================================

  Filter::Filter() :
    filter_{STATE_DIM, MEASUREMENT_DIM, 0.3, 2.0, 0}
  {
    filter_.set_x(MatrixXd::Zero(STATE_DIM, 1));
    filter_.set_P(MatrixXd::Identity(STATE_DIM, STATE_DIM));
    filter_.set_Q(MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.0001);

    // State transition function
    filter_.set_f_fn([](const double dt, const MatrixXd &u, Ref<MatrixXd> x)
                     {
                       // Ignore u
                       // ax and ay are discovered
                       // TODO add z
                       // TODO add rpy -- angles!
                       // TODO implement drag

                       // vx += ax * dt
                       x(1, 0) += x(2, 0) * dt;

                       // x += vx * dt
                       x(0, 0) += x(1, 0) * dt;

                       // vy += ay * dt
                       x(4, 0) += x(5, 0) * dt;

                       // y += vy * dt
                       x(3, 0) += x(4, 0) * dt;
                     });

    // Measurement function
    filter_.set_h_fn([](const Ref<const MatrixXd> &x, Ref<MatrixXd> z)
                     {
                       // TODO add z, rpy

                       // x
                       z(0, 0) = x(0, 0);

                       // y
                       z(1, 0) = x(3, 0);
                     });

  }

  void Filter::filter_odom(const nav_msgs::msg::Odometry::SharedPtr msg,
                           geometry_msgs::msg::PoseWithCovarianceStamped &pose)
  {
    // TODO predict & update from odom
  }

  void Filter::filter_baro(const orca_msgs::msg::Barometer::SharedPtr msg,
                           geometry_msgs::msg::PoseWithCovarianceStamped &pose)
  {
    // TODO predict & update from baro
  }

} // namespace orca_base
