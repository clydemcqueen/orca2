#include "orca_base/filter.hpp"

#include "eigen3/Eigen/Dense"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_base/model.hpp"
#include "orca_base/util.hpp"

namespace orca_base
{
  constexpr int STATE_DIM = 12;       // [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw]T
  constexpr int MEASUREMENT_DIM = 6;  // [x, y, z, roll, pitch, yaw]T
  constexpr int CONTROL_DIM = 4;      // [ax, ay, az, ayaw]T

  //==================================================================
  // Utility functions
  //==================================================================

  // Create control matrix u
  void to_u(const Acceleration &in, Eigen::MatrixXd &out)
  {
    out = Eigen::MatrixXd(CONTROL_DIM, 1);
    out << in.x, in.y, in.z, in.yaw;
  }

  // Create measurement matrix z
  void to_z(const geometry_msgs::msg::Pose &in, Eigen::MatrixXd &out)
  {
    tf2::Transform t_map_base;
    tf2::fromMsg(in, t_map_base);

    tf2Scalar roll, pitch, yaw;
    t_map_base.getBasis().getRPY(roll, pitch, yaw);

    out = Eigen::MatrixXd(MEASUREMENT_DIM, 1);
    out << t_map_base.getOrigin().x(), t_map_base.getOrigin().y(), t_map_base.getOrigin().z(), roll, pitch, yaw;
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

  // Extract pose from state
  void pose_from_x(const Eigen::MatrixXd &in, geometry_msgs::msg::Pose &out)
  {
    out.position.x = in(0, 0);
    out.position.y = in(1, 0);
    out.position.z = in(2, 0);

    tf2::Matrix3x3 m;
    m.setRPY(in(3, 0), in(4, 0), in(5, 0));

    tf2::Quaternion q;
    m.getRotation(q);

    out.orientation = tf2::toMsg(q);
  }

  // Extract twist from state
  void twist_from_x(const Eigen::MatrixXd &in, geometry_msgs::msg::Twist &out)
  {
    out.linear.x = in(6, 0);
    out.linear.y = in(7, 0);
    out.linear.z = in(8, 0);

    out.angular.x = in(9, 0);
    out.angular.y = in(10, 0);
    out.angular.z = in(11, 0);
  }

  // Extract pose covariance
  void pose_covar_from_P(const Eigen::MatrixXd &in, std::array<double, 36> &out)
  {
    for (int i = 0; i < MEASUREMENT_DIM; i++) {
      for (int j = 0; j < MEASUREMENT_DIM; j++) {
        out[i * MEASUREMENT_DIM + j] = in(i, j);  // [0:6, 0:6]
      }
    }
  }

  // Extract twist covariance
  void twist_covar_from_P(const Eigen::MatrixXd &in, std::array<double, 36> &out)
  {
    for (int i = 0; i < MEASUREMENT_DIM; i++) {
      for (int j = 0; j < MEASUREMENT_DIM; j++) {
        out[i * MEASUREMENT_DIM + j] = in(i + MEASUREMENT_DIM, j + MEASUREMENT_DIM);  // [6:12, 6:12]
      }
    }
  }


  //==================================================================
  // Filter
  //==================================================================

  Filter::Filter() :
    filter_{STATE_DIM, MEASUREMENT_DIM, 0.3, 2.0, 0}
  {
    filter_.set_x(Eigen::MatrixXd::Zero(STATE_DIM, 1));
    filter_.set_P(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM));
    filter_.set_Q(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.0001);

    // State transition function
    filter_.set_f_fn([](const double dt, const Eigen::MatrixXd &u, Eigen::Ref<Eigen::MatrixXd> x)
                     {
                       // Acceleration, e.g., ax = thrust_ax - vx * vx * drag_constant
                       // The ROV is self-righting, so predict 0 acceleration for roll and pitch
                       double ax = u(0, 0) - drag_accel_x(x(6, 0));
                       double ay = u(1, 0) - drag_accel_y(x(7, 0));
                       double az = u(2, 0) - drag_accel_z(x(8, 0));
                       double aroll = 0;
                       double apitch = 0;
                       double ayaw = u(3, 0) - drag_accel_yaw(x(11, 0));

                       // The control input makes the filter unstable.
                       // For now predict 0 acceleration.
                       // TODO u(2, 0) includes acceleration due to gravity; remove this
                       // TODO re-enable control input
                       ax = ay = az = ayaw = 0;

                       // Velocity, e.g., vx += ax * dt
                       x(6, 0) += ax * dt;
                       x(7, 0) += ay * dt;
                       x(8, 0) += az * dt;
                       x(9, 0) += aroll * dt;
                       x(10, 0) += apitch * dt;
                       x(11, 0) += ayaw * dt;

                       // Position, e.g., x += vx * dt
                       x(0, 0) += x(6, 0) * dt;
                       x(1, 0) += x(7, 0) * dt;
                       x(2, 0) += x(8, 0) * dt;
                       x(3, 0) = norm_angle(x(3, 0) + x(9, 0) * dt);
                       x(4, 0) = norm_angle(x(4, 0) + x(10, 0) * dt);
                       x(5, 0) = norm_angle(x(5, 0) + x(11, 0) * dt);
                     });

    // Measurement function
    filter_.set_h_fn([](const Eigen::Ref<const Eigen::MatrixXd> &x, Eigen::Ref<Eigen::MatrixXd> z)
                     {
                       z(0, 0) = x(0, 0);
                       z(1, 0) = x(1, 0);
                       z(2, 0) = x(2, 0);
                       z(3, 0) = x(3, 0);
                       z(4, 0) = x(4, 0);
                       z(5, 0) = x(5, 0);
                     });
  }

  void Filter::filter_odom(double dt, const Acceleration &u_bar,
                           const nav_msgs::msg::Odometry &fiducial_odom,
                           nav_msgs::msg::Odometry &filtered_odom)
  {
    Eigen::MatrixXd u;
    to_u(u_bar, u);

    filter_.predict(dt, u);

    Eigen::MatrixXd z;
    to_z(fiducial_odom.pose.pose, z);

    Eigen::MatrixXd R;
    to_R(fiducial_odom.pose.covariance, R);

    filter_.update(z, R);

    filtered_odom.header = fiducial_odom.header;
    pose_from_x(filter_.x(), filtered_odom.pose.pose);
    twist_from_x(filter_.x(), filtered_odom.twist.twist);
    pose_covar_from_P(filter_.P(), filtered_odom.pose.covariance);
    twist_covar_from_P(filter_.P(), filtered_odom.twist.covariance);
  }

} // namespace orca_base
