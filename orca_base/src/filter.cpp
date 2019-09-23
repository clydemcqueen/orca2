#include "orca_base/filter.hpp"

#include "eigen3/Eigen/Dense"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_base/model.hpp"
#include "orca_base/util.hpp"

namespace orca_base
{
  constexpr int STATE_DIM = 18;       // [x, y, ..., vx, vy, ..., ax, ay, ...]T
  constexpr int BARO_DIM = 1;         // [z]T
  constexpr int ODOM_DIM = 6;         // [x, y, z, roll, pitch, yaw]T
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

  bool valid_stamp(const rclcpp::Time &stamp)
  {
    return stamp.nanoseconds() > 0;
  }

  // Create control matrix u
  void to_u(const Acceleration &in, Eigen::MatrixXd &out)
  {
    out = Eigen::MatrixXd(CONTROL_DIM, 1);
    out << in.x, in.y, in.z, in.yaw;
  }

  // Create measurement matrix z
  void odom_to_z(const nav_msgs::msg::Odometry &odom, Eigen::MatrixXd &z)
  {
    tf2::Transform t_map_base;
    tf2::fromMsg(odom.pose.pose, t_map_base);

    tf2Scalar roll, pitch, yaw;
    t_map_base.getBasis().getRPY(roll, pitch, yaw);

    z = Eigen::MatrixXd(ODOM_DIM, 1);
    z << t_map_base.getOrigin().x(), t_map_base.getOrigin().y(), t_map_base.getOrigin().z(), roll, pitch, yaw;
  }

  void baro_to_z(const orca_msgs::msg::Barometer &baro, Eigen::MatrixXd &z)
  {
    z = Eigen::MatrixXd(BARO_DIM, 1);
    z << -baro.depth;
  }

  // Create measurement covariance matrix R
  void odom_to_R(const nav_msgs::msg::Odometry &odom, Eigen::MatrixXd &R)
  {
    R = Eigen::MatrixXd(ODOM_DIM, ODOM_DIM);
    for (int i = 0; i < ODOM_DIM; i++) {
      for (int j = 0; j < ODOM_DIM; j++) {
        R(i, j) = odom.pose.covariance[i * ODOM_DIM + j];
      }
    }
  }

  void baro_to_R(const orca_msgs::msg::Barometer &baro, Eigen::MatrixXd &R)
  {
    R = Eigen::MatrixXd(BARO_DIM, BARO_DIM);
    R << 0.01; // Same as orca_gazebo/barometer_plugin.cpp
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
  void pose_covar_from_P(const Eigen::MatrixXd &P, std::array<double, 36> &pose_covar)
  {
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        pose_covar[i * 6 + j] = P(i, j);  // [0:6, 0:6]
      }
    }
  }

  // Extract twist covariance
  void twist_covar_from_P(const Eigen::MatrixXd &P, std::array<double, 36> &twist_covar)
  {
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        twist_covar[i * 6 + j] = P(i + 6, j + 6);  // [6:12, 6:12]
      }
    }
  }

  //==================================================================
  // Filter
  //==================================================================

  Filter::Filter(const BaseContext &cxt_) :
    filter_{STATE_DIM, 0.3, 2.0, 0}
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

    // TODO set residual & mean functions for x
  }

  double Filter::dt(const rclcpp::Time &stamp)
  {
    // Compute delta from last message
    // Use a small delta to bootstrap the filter
    // TODO don't call predict() if dt < epsilon
    double result = valid_stamp(filter_time_) ? (stamp - filter_time_).seconds() : 0.1;

    // Move time forward
    filter_time_ = stamp;

    return result;
  }

  void Filter::process_baro(const Acceleration &u_bar, const orca_msgs::msg::Barometer &baro,
                            nav_msgs::msg::Odometry &filtered_odom)
  {
    Eigen::MatrixXd u;
    to_u(u_bar, u);

    filter_.predict(dt(baro.header.stamp), u);

    Eigen::MatrixXd z;
    baro_to_z(baro, z);

    Eigen::MatrixXd R;
    baro_to_R(baro, R);

    // Measurement function
    filter_.set_h_fn([](const Eigen::Ref<const Eigen::MatrixXd> &x, Eigen::Ref<Eigen::MatrixXd> z)
                     {
                       z(0, 0) = x(3, 0);
                     });

    // TODO set residual & mean functions for z

    filter_.update(z, R);

    filtered_odom.header = baro.header;
    pose_from_x(filter_.x(), filtered_odom.pose.pose);
    twist_from_x(filter_.x(), filtered_odom.twist.twist);
    pose_covar_from_P(filter_.P(), filtered_odom.pose.covariance);
    twist_covar_from_P(filter_.P(), filtered_odom.twist.covariance);
  }

  void Filter::process_odom(const Acceleration &u_bar, const nav_msgs::msg::Odometry &odom,
                            nav_msgs::msg::Odometry &filtered_odom)
  {
    Eigen::MatrixXd u;
    to_u(u_bar, u);

    filter_.predict(dt(odom.header.stamp), u);

    Eigen::MatrixXd z;
    odom_to_z(odom, z);

    Eigen::MatrixXd R;
    odom_to_R(odom, R);

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

    // TODO set residual & mean functions for z

    filter_.update(z, R);

    filtered_odom.header = odom.header;
    pose_from_x(filter_.x(), filtered_odom.pose.pose);
    twist_from_x(filter_.x(), filtered_odom.twist.twist);
    pose_covar_from_P(filter_.P(), filtered_odom.pose.covariance);
    twist_covar_from_P(filter_.P(), filtered_odom.twist.covariance);
  }

  void Filter::queue_baro(rclcpp::Logger logger, const orca_msgs::msg::Barometer &baro)
  {
    rclcpp::Time baro_stamp{baro.header.stamp};

    if (!valid_stamp(baro_stamp)) {
      RCLCPP_WARN(logger, "barometer message has invalid time, dropping");
    } else {
      baro_q_ = baro;
    }
  }

  bool Filter::filter_odom(rclcpp::Logger logger, const Acceleration &u_bar, const nav_msgs::msg::Odometry &odom,
                           nav_msgs::msg::Odometry &filtered_odom)
  {
    rclcpp::Time odom_stamp{odom.header.stamp};

    if (!valid_stamp(odom_stamp)) {
      RCLCPP_WARN(logger, "odometry message has invalid time, dropping");
      return false;
    }

    if (valid_stamp(filter_time_) && valid_stamp(odom_stamp) && odom_stamp < filter_time_) {
      RCLCPP_WARN(logger, "odometry message older than filter time, dropping");
      return false;
    }

    rclcpp::Time baro_stamp{baro_q_.header.stamp};

    if (valid_stamp(filter_time_) && valid_stamp(baro_stamp) && baro_stamp < filter_time_) {
      RCLCPP_WARN(logger, "barometer message older than filter time, dropping");
      baro_q_ = orca_msgs::msg::Barometer{};
      baro_stamp = rclcpp::Time{baro_q_.header.stamp};
    }

    if (valid_stamp(baro_stamp)) {
      if (baro_stamp < odom_stamp) {
        process_baro(u_bar, baro_q_, filtered_odom);
        process_odom(u_bar, odom, filtered_odom);
      } else {
        process_odom(u_bar, odom, filtered_odom);
        process_baro(u_bar, baro_q_, filtered_odom);
      }

      baro_q_ = orca_msgs::msg::Barometer{};
    } else {
      process_odom(u_bar, odom, filtered_odom);
    }

    return filter_valid();
  }

} // namespace orca_base
