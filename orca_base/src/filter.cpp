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
  constexpr double MIN_DT = 0.001;
  constexpr double MAX_DT = 1.0;

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
    z << baro.z;
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
    R << baro.z_variance;
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
  // Unscented residual and mean functions for state (x) and odom measurement (z)
  //
  // Because of the layout of the state and odom measurement matrices and the way that Eigen works
  // these functions can serve double-duty.
  //
  // x:       [x, y, z, r, p, y, ...]
  // odom z:  [x, y, z, r, p, y]
  //
  // The mean function needs to compute the mean of angles, which doesn't have a precise meaning.
  // See https://en.wikipedia.org/wiki/Mean_of_circular_quantities for the method used here.
  //==================================================================

  Eigen::MatrixXd orca_state_residual(const Eigen::Ref<const Eigen::MatrixXd> &x, const Eigen::MatrixXd &mean)
  {
    // Residual for all fields
    Eigen::MatrixXd residual = x - mean;

    // Normalize roll, pitch and yaw
    residual(3, 0) = norm_angle(residual(3, 0));
    residual(4, 0) = norm_angle(residual(4, 0));
    residual(5, 0) = norm_angle(residual(5, 0));

    return residual;
  }

  Eigen::MatrixXd orca_state_mean(const Eigen::MatrixXd &sigma_points, const Eigen::MatrixXd &Wm)
  {
    Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(sigma_points.rows(), 1);

    // Standard mean for all fields
    for (long i = 0; i < sigma_points.cols(); ++i) {
      mean += Wm(0, i) * sigma_points.col(i);
    }

    // Sum the sines and cosines
    double sum_r_sin = 0.0, sum_r_cos = 0.0;
    double sum_p_sin = 0.0, sum_p_cos = 0.0;
    double sum_y_sin = 0.0, sum_y_cos = 0.0;

    for (long i = 0; i < sigma_points.cols(); ++i) {
      sum_r_sin += Wm(0, i) * sin(sigma_points(3, i));
      sum_r_cos += Wm(0, i) * cos(sigma_points(3, i));

      sum_p_sin += Wm(0, i) * sin(sigma_points(4, i));
      sum_p_cos += Wm(0, i) * cos(sigma_points(4, i));

      sum_y_sin += Wm(0, i) * sin(sigma_points(5, i));
      sum_y_cos += Wm(0, i) * cos(sigma_points(5, i));
    }

    // Mean is arctan2 of the sums
    mean(3, 0) = atan2(sum_r_sin, sum_r_cos);
    mean(4, 0) = atan2(sum_p_sin, sum_p_cos);
    mean(5, 0) = atan2(sum_y_sin, sum_y_cos);

    return mean;
  }

//==================================================================
  // Filter
  //==================================================================

  Filter::Filter(const rclcpp::Logger &logger, const BaseContext &cxt_) :
    logger_{logger},
    filter_{STATE_DIM, 0.3, 2.0, 0}
  {
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

    // State residual and mean functions
    filter_.set_r_x_fn(orca_state_residual);
    filter_.set_mean_x_fn(orca_state_mean);
  }

  void Filter::predict(const Acceleration &u_bar, const rclcpp::Time &stamp)
  {
    // Compute delta from last message
    // Use a small delta to bootstrap the filter
    double dt = valid_stamp(filter_time_) ? (stamp - filter_time_).seconds() : 0.1;

    if (dt < MIN_DT) {
      RCLCPP_DEBUG(logger_, "dt %g is too small, skip predict", dt);
    } else if (dt > MAX_DT) {
      RCLCPP_WARN(logger_, "dt %g is too large, skip predict", dt);

      // Catch up
      filter_time_ = stamp;
    } else {
      Eigen::MatrixXd u;
      to_u(u_bar, u);
      filter_.predict(dt, u);
      filter_time_ = stamp;
    }
  }

  void Filter::update(const Eigen::MatrixXd &z, const Eigen::MatrixXd &R, const builtin_interfaces::msg::Time &stamp,
                      nav_msgs::msg::Odometry &filtered_odom)
  {
    filter_.update(z, R);

    filtered_odom.header.stamp = stamp;
    pose_from_x(filter_.x(), filtered_odom.pose.pose);
    twist_from_x(filter_.x(), filtered_odom.twist.twist);
    pose_covar_from_P(filter_.P(), filtered_odom.pose.covariance);
    twist_covar_from_P(filter_.P(), filtered_odom.twist.covariance);
  }

  void Filter::process_baro(const Acceleration &u_bar, const orca_msgs::msg::Barometer &baro,
                            nav_msgs::msg::Odometry &filtered_odom)
  {
    predict(u_bar, baro.header.stamp);

    Eigen::MatrixXd z;
    baro_to_z(baro, z);

    Eigen::MatrixXd R;
    baro_to_R(baro, R);

    // Measurement function
    filter_.set_h_fn([](const Eigen::Ref<const Eigen::MatrixXd> &x, Eigen::Ref<Eigen::MatrixXd> z)
                     {
                       z(0, 0) = x(2, 0);
                     });

    // Use the standard residual and mean functions for barometer readings
    filter_.set_r_z_fn(ukf::residual);
    filter_.set_mean_z_fn(ukf::unscented_mean);

    update(z, R, baro.header.stamp, filtered_odom);
  }

  void Filter::process_odom(const Acceleration &u_bar, const nav_msgs::msg::Odometry &odom,
                            nav_msgs::msg::Odometry &filtered_odom)
  {
    predict(u_bar, odom.header.stamp);

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

    // Use the custom state residual and mean functions for fiducial_vlam odometry
    filter_.set_r_z_fn(orca_state_residual);
    filter_.set_mean_z_fn(orca_state_mean);

    update(z, R, odom.header.stamp, filtered_odom);
  }

  void Filter::queue_baro(const orca_msgs::msg::Barometer &baro)
  {
    rclcpp::Time baro_stamp{baro.header.stamp};

    if (!valid_stamp(baro_stamp)) {
      RCLCPP_WARN(logger_, "barometer message has invalid time, dropping");
    } else {
      baro_q_ = baro;
    }
  }

  bool Filter::filter_odom(const Acceleration &u_bar, const nav_msgs::msg::Odometry &odom,
                           nav_msgs::msg::Odometry &filtered_odom)
  {
    rclcpp::Time odom_stamp{odom.header.stamp};

    if (!valid_stamp(odom_stamp)) {
      RCLCPP_WARN(logger_, "odometry message has invalid time, dropping");
      return true;
    }

    if (valid_stamp(filter_time_) && valid_stamp(odom_stamp) && odom_stamp < filter_time_) {
      RCLCPP_WARN(logger_, "odometry message older than filter time, dropping");
      return true;
    }

    rclcpp::Time baro_stamp{baro_q_.header.stamp};

    if (valid_stamp(filter_time_) && valid_stamp(baro_stamp) && baro_stamp < filter_time_) {
      RCLCPP_WARN(logger_, "barometer message older than filter time, dropping");
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
