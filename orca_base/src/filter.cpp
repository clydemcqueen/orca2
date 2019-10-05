#include "orca_base/filter.hpp"

#include "eigen3/Eigen/Dense"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_base/model.hpp"
#include "orca_base/util.hpp"

namespace orca_base
{

  //=============================================================================
  // Constants
  //=============================================================================

  constexpr int STATE_DIM = 18;       // [x, y, ..., vx, vy, ..., ax, ay, ...]T
  constexpr int BARO_DIM = 1;         // [z]T
  constexpr int POSE_DIM = 6;         // [x, y, z, roll, pitch, yaw]T
  constexpr int CONTROL_DIM = 4;      // [ax, ay, az, ayaw]T

  constexpr double MIN_DT = 0.001;
  constexpr double DEFAULT_DT = 0.1;
  constexpr double MAX_DT = 1.0;

  // Maximum predicted acceleration
  // The AUV may be tossed around by waves or bump into something
  constexpr double MAX_PREDICTED_ACCEL_XYZ = 100;
  constexpr double MAX_PREDICTED_ACCEL_RPY = 100;

  // Maximum predicted velocity in water
  constexpr double MAX_PREDICTED_VELO_XYZ = 100;
  constexpr double MAX_PREDICTED_VELO_RPY = 100;

#define x_x x(0, 0)
#define x_y x(1, 0)
#define x_z x(2, 0)
#define x_roll x(3, 0)
#define x_pitch x(4, 0)
#define x_yaw x(5, 0)
#define x_vx x(6, 0)
#define x_vy x(7, 0)
#define x_vz x(8, 0)
#define x_vroll x(9, 0)
#define x_vpitch x(10, 0)
#define x_vyaw x(11, 0)
#define x_ax x(12, 0)
#define x_ay x(13, 0)
#define x_az x(14, 0)
#define x_aroll x(15, 0)
#define x_apitch x(16, 0)
#define x_ayaw x(17, 0)

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
  void pose_to_z(const geometry_msgs::msg::PoseWithCovarianceStamped &pose, Eigen::MatrixXd &z)
  {
    tf2::Transform t_map_base;
    tf2::fromMsg(pose.pose.pose, t_map_base);

    tf2Scalar roll, pitch, yaw;
    t_map_base.getBasis().getRPY(roll, pitch, yaw);

    z = Eigen::MatrixXd(POSE_DIM, 1);
    z << t_map_base.getOrigin().x(), t_map_base.getOrigin().y(), t_map_base.getOrigin().z(), roll, pitch, yaw;
  }

  void depth_to_z(const orca_msgs::msg::Depth &depth, Eigen::MatrixXd &z)
  {
    z = Eigen::MatrixXd(BARO_DIM, 1);
    z << depth.z;
  }

  // Create measurement covariance matrix R
  void pose_to_R(const geometry_msgs::msg::PoseWithCovarianceStamped &pose, Eigen::MatrixXd &R)
  {
    R = Eigen::MatrixXd(POSE_DIM, POSE_DIM);
    for (int i = 0; i < POSE_DIM; i++) {
      for (int j = 0; j < POSE_DIM; j++) {
        R(i, j) = pose.pose.covariance[i * POSE_DIM + j];
      }
    }
  }

  void depth_to_R(const orca_msgs::msg::Depth &depth, Eigen::MatrixXd &R)
  {
    R = Eigen::MatrixXd(BARO_DIM, BARO_DIM);
    R << depth.z_variance;
  }

  // Extract pose from state
  void pose_from_x(const Eigen::MatrixXd &x, geometry_msgs::msg::Pose &out)
  {
    out.position.x = x_x;
    out.position.y = x_y;
    out.position.z = x_z;

    tf2::Matrix3x3 m;
    m.setRPY(x_roll, x_pitch, x_yaw);

    tf2::Quaternion q;
    m.getRotation(q);

    out.orientation = tf2::toMsg(q);
  }

  // Extract twist from state
  void twist_from_x(const Eigen::MatrixXd &x, geometry_msgs::msg::Twist &out)
  {
    out.linear.x = x_vx;
    out.linear.y = x_vy;
    out.linear.z = x_vz;

    out.angular.x = x_vroll;
    out.angular.y = x_vpitch;
    out.angular.z = x_vyaw;
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

  Filter::Filter(const rclcpp::Logger &logger, const FilterContext &cxt) :
    logger_{logger},
    depth_q_{logger},
    pose_q_{logger},
    filter_{STATE_DIM, 0.3, 2.0, 0}
  {
    filter_.set_Q(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.01);

    // State transition function
    filter_.set_f_fn([&cxt](const double dt, const Eigen::MatrixXd &u, Eigen::Ref<Eigen::MatrixXd> x)
                     {
                       if (cxt.predict_accel_) {
                         // Assume 0 acceleration
                         x_ax = 0;
                         x_ay = 0;
                         x_az = 0;
                         x_aroll = 0;
                         x_apitch = 0;
                         x_ayaw = 0;

                         if (cxt.predict_accel_control_) {
                           // Add acceleration due to control
                           x_ax += u(0, 0);
                           x_ay += u(1, 0);
                           x_az += u(2, 0);
                           x_ayaw += u(3, 0);
                         }

                         if (cxt.predict_accel_drag_) {
                           // Add acceleration due to drag
                           // TODO create & use AddLinkForce(drag_force, c_of_mass) and AddRelativeTorque(drag_torque)
                           // Simple approximation:
                           x_ax += cxt.model_.drag_accel_x(x_vx);
                           x_ay += cxt.model_.drag_accel_y(x_vy);
                           x_az += cxt.model_.drag_accel_z(x_vz);
                           x_aroll += cxt.model_.drag_accel_yaw(x_vroll);
                           x_apitch += cxt.model_.drag_accel_yaw(x_vpitch);
                           x_ayaw += cxt.model_.drag_accel_yaw(x_vyaw);
                         }

                         if (cxt.predict_accel_buoyancy_) {
                           // Add acceleration due to gravity and buoyancy
                           // TODO create & use AddLinkForce(buoyancy_force, c_of_volume)
                           // Simple approximation:
                           x_roll = 0;
                           x_pitch = 0;
                           x_az -= cxt.model_.hover_accel_z();
                         }
                       }

                       // Clamp acceleration
                       x_ax = clamp(x_ax, MAX_PREDICTED_ACCEL_XYZ);
                       x_ay = clamp(x_ay, MAX_PREDICTED_ACCEL_XYZ);
                       x_az = clamp(x_az, MAX_PREDICTED_ACCEL_XYZ);
                       x_aroll = clamp(x_aroll, MAX_PREDICTED_ACCEL_RPY);
                       x_apitch = clamp(x_apitch, MAX_PREDICTED_ACCEL_RPY);
                       x_ayaw = clamp(x_ayaw, MAX_PREDICTED_ACCEL_RPY);

                       // Velocity, vx += ax * dt
                       x_vx += x_ax * dt;
                       x_vy += x_ay * dt;
                       x_vz += x_az * dt;
                       x_vroll += x_aroll * dt;
                       x_vpitch += x_apitch * dt;
                       x_vyaw += x_ayaw * dt;

                       // Clamp velocity
                       x_vx = clamp(x_vx, MAX_PREDICTED_VELO_XYZ);
                       x_vy = clamp(x_vy, MAX_PREDICTED_VELO_XYZ);
                       x_vz = clamp(x_vz, MAX_PREDICTED_VELO_XYZ);
                       x_vroll = clamp(x_vroll, MAX_PREDICTED_VELO_RPY);
                       x_vpitch = clamp(x_vpitch, MAX_PREDICTED_VELO_RPY);
                       x_vyaw = clamp(x_vyaw, MAX_PREDICTED_VELO_RPY);

                       // Position, x += vx * dt
                       x_x += x_vx * dt;
                       x_y += x_vy * dt;
                       x_z += x_vz * dt;
                       x_roll = norm_angle(x_roll + x_vroll * dt);
                       x_pitch = norm_angle(x_pitch + x_vpitch * dt);
                       x_yaw = norm_angle(x_yaw + x_vyaw * dt);
                     });

    // State residual and mean functions
    filter_.set_r_x_fn(orca_state_residual);
    filter_.set_mean_x_fn(orca_state_mean);
  }

  void Filter::predict(const rclcpp::Time &stamp, const Acceleration &u_bar)
  {
    // Filter time starts at 0, test for this
    if (valid_stamp(filter_time_)) {
      // Compute delta from last message, must be zero or positive
      double dt = (stamp - filter_time_).seconds();
      assert(dt >= 0);

      if (dt > MAX_DT) {
        // Delta is too large for some reason, use a smaller delta to avoid screwing up the filter
        RCLCPP_WARN(logger_, "dt %g is too large, use default %g", dt, DEFAULT_DT);
        dt = DEFAULT_DT;
      }

      if (dt < MIN_DT) {
        // Delta is quite small, possibly 0
        RCLCPP_DEBUG(logger_, "skip predict: msg=%s filter=%s", to_str(stamp).c_str(), to_str(filter_time_).c_str());
      } else {
        // Run the prediction
        RCLCPP_DEBUG(logger_, "predict: msg=%s filter=%s", to_str(stamp).c_str(), to_str(filter_time_).c_str());
        Eigen::MatrixXd u;
        to_u(u_bar, u);
        filter_.predict(dt, u);
      }
    }

    // Always advance filter_time_
    filter_time_ = stamp;
  }

  void Filter::process_depth(const Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom)
  {
    orca_msgs::msg::Depth depth = depth_q_.pop();
    RCLCPP_DEBUG(logger_, "process pose: %s", to_str(depth.header.stamp).c_str());
    predict(depth.header.stamp, u_bar);

    Eigen::MatrixXd z;
    depth_to_z(depth, z);

    Eigen::MatrixXd R;
    depth_to_R(depth, R);

    // Measurement function
    filter_.set_h_fn([](const Eigen::Ref<const Eigen::MatrixXd> &x, Eigen::Ref<Eigen::MatrixXd> z)
                     {
                       z(0, 0) = x_z;
                     });

    // Use the standard residual and mean functions for depth readings
    filter_.set_r_z_fn(ukf::residual);
    filter_.set_mean_z_fn(ukf::unscented_mean);

    filter_.update(z, R);
  }

  void Filter::process_pose(const Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose = pose_q_.pop();
    RCLCPP_DEBUG(logger_, "process pose: %s", to_str(pose.header.stamp).c_str());
    predict(pose.header.stamp, u_bar);

    Eigen::MatrixXd z;
    pose_to_z(pose, z);

    Eigen::MatrixXd R;
    pose_to_R(pose, R);

    // Measurement function
    filter_.set_h_fn([](const Eigen::Ref<const Eigen::MatrixXd> &x, Eigen::Ref<Eigen::MatrixXd> z)
                     {
                       z(0, 0) = x_x;
                       z(1, 0) = x_y;
                       z(2, 0) = x_z;
                       z(3, 0) = x_roll;
                       z(4, 0) = x_pitch;
                       z(5, 0) = x_yaw;
                     });

    // Use the custom state residual and mean functions for fiducial_vlam odometry
    filter_.set_r_z_fn(orca_state_residual);
    filter_.set_mean_z_fn(orca_state_mean);

    filter_.update(z, R);
  }

  void Filter::queue_depth(const orca_msgs::msg::Depth &depth)
  {
    rclcpp::Time stamp{depth.header.stamp};

    if (stamp >= filter_time_) {
      RCLCPP_DEBUG(logger_, "queue depth message %s", to_str(stamp).c_str());
      depth_q_.push(depth);
    } else {
      RCLCPP_WARN(logger_, "depth message %s is older than filter time %s, dropping", to_str(stamp).c_str(),
                  to_str(filter_time_).c_str());
    }
  }

  void Filter::queue_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose)
  {
    rclcpp::Time stamp{pose.header.stamp};

    if (stamp >= filter_time_) {
      RCLCPP_DEBUG(logger_, "queue pose message %s", to_str(stamp).c_str());
      pose_q_.push(pose);
    } else {
      RCLCPP_WARN(logger_, "pose message %s is older than filter time %s, dropping", to_str(stamp).c_str(),
                  to_str(filter_time_).c_str());
    }
  }

  bool Filter::process(const rclcpp::Time &t, const Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom)
  {
    rclcpp::Time too_old = t - TOO_OLD;
    rclcpp::Time start = filter_time_ > too_old ? filter_time_ : too_old;
    rclcpp::Time end = t - LAG;

    rclcpp::Time depth_stamp, pose_stamp;
    bool depth_msg_ready = depth_q_.msg_ready(start, end, depth_stamp);
    bool pose_msg_ready = pose_q_.msg_ready(start, end, pose_stamp);

    if (!depth_msg_ready && !pose_msg_ready) {
      // No measurements, just predict
      RCLCPP_DEBUG(logger_, "just predict: %s", to_str(t).c_str());
      predict(end, u_bar);
    } else {
      // Process all measurements in order
      while (depth_msg_ready || pose_msg_ready) {
        if (depth_msg_ready && pose_msg_ready) {
          if (depth_stamp < pose_stamp) {
            process_depth(u_bar, filtered_odom);
          } else {
            process_pose(u_bar, filtered_odom);
          }
        } else if (depth_msg_ready) {
          process_depth(u_bar, filtered_odom);
        } else {
          process_pose(u_bar, filtered_odom);
        }

        depth_msg_ready = depth_q_.msg_ready(start, end, depth_stamp);
        pose_msg_ready = pose_q_.msg_ready(start, end, pose_stamp);
      }
    }

    bool valid = filter_.valid();
    if (!valid) {
      // Crude restart, useful for debugging the filter
      RCLCPP_ERROR(logger_, "Restart filter");
      filter_.set_P(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM));
      valid = true;
    }

    // Return the best estimate
    filtered_odom.header.stamp = filter_time_;
    pose_from_x(filter_.x(), filtered_odom.pose.pose);
    twist_from_x(filter_.x(), filtered_odom.twist.twist);
    pose_covar_from_P(filter_.P(), filtered_odom.pose.covariance);
    twist_covar_from_P(filter_.P(), filtered_odom.twist.covariance);

    return valid;
  }

} // namespace orca_base
