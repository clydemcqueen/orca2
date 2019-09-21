#include "orca_base/filter.hpp"

#include "eigen3/Eigen/Dense"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_base/model.hpp"
#include "orca_base/util.hpp"

namespace orca_base
{
  constexpr int STATE_DIM = 18;       // [x, y, ..., vx, vy, ..., ax, ay, ...]T
  constexpr int MEASUREMENT_DIM = 6;  // [x, y, z, roll, pitch, yaw]T
  constexpr int CONTROL_DIM = 4;      // [ax, ay, az, ayaw]T

  // Maximum predicted acceleration
  // The AUV may be tossed around by waves or bump into something
  constexpr double MAX_PREDICTED_ACCEL_XYZ = 100;
  constexpr double MAX_PREDICTED_ACCEL_RPY = 100;

  // Maximum predicted velocity in water
  constexpr double MAX_PREDICTED_VELO_XYZ = 100;
  constexpr double MAX_PREDICTED_VELO_RPY = 100;

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

  Filter::Filter(const BaseContext &cxt_) :
    filter_{STATE_DIM, MEASUREMENT_DIM, 0.3, 2.0, 0}
  {
    filter_.set_x(Eigen::MatrixXd::Zero(STATE_DIM, 1));
    filter_.set_P(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM));
    filter_.set_Q(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.01);

    // State transition function
    filter_.set_f_fn([&cxt_](const double dt, const Eigen::MatrixXd &u, Eigen::Ref<Eigen::MatrixXd> x)
                     {
                       if (cxt_.filter_predict_accel_) {
                         // Assume 0 acceleration
                         x(12, 0) = 0;
                         x(13, 0) = 0;
                         x(14, 0) = 0;
                         x(15, 0) = 0;
                         x(16, 0) = 0;
                         x(17, 0) = 0;

                         if (cxt_.filter_predict_control_) {
                           // Add acceleration due to control
                           // Back out acceleration due to gravity & buoyancy
                           x(12, 0) += u(0, 0);
                           x(13, 0) += u(1, 0);
                           x(14, 0) += u(2, 0) - HOVER_ACCEL_Z;
                           x(17, 0) += u(3, 0);
                         }

                         if (cxt_.filter_predict_drag_) {
                           // Add acceleration due to drag
                           x(12, 0) += drag_accel_x(x(6, 0));
                           x(13, 0) += drag_accel_y(x(7, 0));
                           x(14, 0) += drag_accel_z(x(8, 0));
                           x(17, 0) += drag_accel_yaw(x(11, 0));
                         }
                       }

                       // Clamp acceleration
                       x(12, 0) = clamp(x(12, 0), MAX_PREDICTED_ACCEL_XYZ);
                       x(13, 0) = clamp(x(13, 0), MAX_PREDICTED_ACCEL_XYZ);
                       x(14, 0) = clamp(x(14, 0), MAX_PREDICTED_ACCEL_XYZ);
                       x(15, 0) = clamp(x(15, 0), MAX_PREDICTED_ACCEL_RPY);
                       x(16, 0) = clamp(x(16, 0), MAX_PREDICTED_ACCEL_RPY);
                       x(17, 0) = clamp(x(17, 0), MAX_PREDICTED_ACCEL_RPY);

                       // Velocity, vx += ax * dt
                       x(6, 0) += x(12, 0) * dt;
                       x(7, 0) += x(13, 0) * dt;
                       x(8, 0) += x(14, 0) * dt;
                       x(9, 0) += x(15, 0) * dt;
                       x(10, 0) += x(16, 0) * dt;
                       x(11, 0) += x(17, 0) * dt;

                       // Clamp velocity
                       x(6, 0) = clamp(x(6, 0), MAX_PREDICTED_VELO_XYZ);
                       x(7, 0) = clamp(x(7, 0), MAX_PREDICTED_VELO_XYZ);
                       x(8, 0) = clamp(x(8, 0), MAX_PREDICTED_VELO_XYZ);
                       x(9, 0) = clamp(x(9, 0), MAX_PREDICTED_VELO_RPY);
                       x(10, 0) = clamp(x(10, 0), MAX_PREDICTED_VELO_RPY);
                       x(11, 0) = clamp(x(11, 0), MAX_PREDICTED_VELO_RPY);

                       // Position, x += vx * dt
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

  bool Filter::filter_odom(double dt, const Acceleration &u_bar,
                           const nav_msgs::msg::Odometry &fiducial_odom,
                           nav_msgs::msg::Odometry &filtered_odom)
  {
    Eigen::MatrixXd u;
    to_u(u_bar, u);

    if (!filter_.predict(dt, u)) {
      return false;
    }

    Eigen::MatrixXd z;
    to_z(fiducial_odom.pose.pose, z);

    Eigen::MatrixXd R;
    to_R(fiducial_odom.pose.covariance, R);

    if (!filter_.update(z, R)) {
      return false;
    }

    filtered_odom.header = fiducial_odom.header;
    pose_from_x(filter_.x(), filtered_odom.pose.pose);
    twist_from_x(filter_.x(), filtered_odom.twist.twist);
    pose_covar_from_P(filter_.P(), filtered_odom.pose.covariance);
    twist_covar_from_P(filter_.P(), filtered_odom.twist.covariance);

    return true;
  }

} // namespace orca_base
