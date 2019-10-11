#include "orca_base/filter_base.hpp"

#include "eigen3/Eigen/Dense"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_base/model.hpp"
#include "orca_base/util.hpp"

namespace orca_base
{

  //==================================================================
  // FourFilter -- 4dof filter with 12 dimensions
  //==================================================================

  constexpr int FOUR_STATE_DIM = 12;      // [x, y, ..., vx, vy, ..., ax, ay, ...]T

  // Four state macros
#define fx_x x(0)
#define fx_y x(1)
#define fx_z x(2)
#define fx_yaw x(3)
#define fx_vx x(4)
#define fx_vy x(5)
#define fx_vz x(6)
#define fx_vyaw x(7)
#define fx_ax x(8)
#define fx_ay x(9)
#define fx_az x(10)
#define fx_ayaw x(11)

  // Extract pose from PoseFilter state
  void pose_from_fx(const Eigen::VectorXd &x, geometry_msgs::msg::Pose &out)
  {
    out.position.x = fx_x;
    out.position.y = fx_y;
    out.position.z = fx_z;

    tf2::Matrix3x3 m;
    m.setRPY(0, 0, fx_yaw);

    tf2::Quaternion q;
    m.getRotation(q);

    out.orientation = tf2::toMsg(q);
  }

  // Extract twist from PoseFilter state
  void twist_from_fx(const Eigen::VectorXd &x, geometry_msgs::msg::Twist &out)
  {
    out.linear.x = fx_vx;
    out.linear.y = fx_vy;
    out.linear.z = fx_vz;

    out.angular.x = 0;
    out.angular.y = 0;
    out.angular.z = fx_vyaw;
  }

  // Extract pose covariance from PoseFilter covariance
  void flatten_4x4_covar(const Eigen::MatrixXd &P, std::array<double, 36> &pose_covar, int offset)
  {
    // Start with identity
    Eigen::MatrixXd m = Eigen::MatrixXd::Identity(6, 6);

    // Copy values from the 4x4
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        m(i < 3 ? i : i + 2, j < 3 ? j : j + 2) = P(i + offset, j + offset);
      }
    }

    // Flatten
    flatten_6x6_covar(m, pose_covar, 0);
  }

  Eigen::VectorXd four_state_residual(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::VectorXd &mean)
  {
    // Residual for all fields
    Eigen::VectorXd residual = x - mean;

    // Normalize yaw
    residual(3) = norm_angle(residual(3));

    return residual;
  }

  Eigen::VectorXd four_state_mean(const Eigen::MatrixXd &sigma_points, const Eigen::RowVectorXd &Wm)
  {
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(sigma_points.rows());

    // Standard mean for all fields
    for (long i = 0; i < sigma_points.cols(); ++i) {
      mean += Wm(i) * sigma_points.col(i);
    }

    // Sum the sines and cosines
    double sum_y_sin = 0.0, sum_y_cos = 0.0;

    for (long i = 0; i < sigma_points.cols(); ++i) {
      sum_y_sin += Wm(i) * sin(sigma_points(3, i));
      sum_y_cos += Wm(i) * cos(sigma_points(3, i));
    }

    // Mean is arctan2 of the sums
    mean(3) = atan2(sum_y_sin, sum_y_cos);

    return mean;
  }

  FourFilter::FourFilter(const rclcpp::Logger &logger, const FilterContext &cxt) :
    FilterBase{logger, cxt, FOUR_STATE_DIM}
  {
    filter_.set_Q(Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 0.01);

    // State transition function
    filter_.set_f_fn(
      [&cxt](const double dt, const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> x)
      {
        if (cxt.predict_accel_) {
          // Assume 0 acceleration
          fx_ax = 0;
          fx_ay = 0;
          fx_az = 0;
          fx_ayaw = 0;

          if (cxt.predict_accel_control_) {
            // Add acceleration due to control
            fx_ax += u(0, 0);
            fx_ay += u(1, 0);
            fx_az += u(2, 0);
            fx_ayaw += u(3, 0);
          }

          if (cxt.predict_accel_drag_) {
            // Add acceleration due to drag
            // TODO create & use AddLinkForce(drag_force, c_of_mass) and AddRelativeTorque(drag_torque)
            // Simple approximation:
            fx_ax += cxt.model_.drag_accel_x(fx_vx);
            fx_ay += cxt.model_.drag_accel_y(fx_vy);
            fx_az += cxt.model_.drag_accel_z(fx_vz);
            fx_ayaw += cxt.model_.drag_accel_yaw(fx_vyaw);
          }

          if (cxt.predict_accel_buoyancy_) {
            // Add acceleration due to gravity and buoyancy
            // TODO create & use AddLinkForce(buoyancy_force, c_of_volume)
            // Simple approximation:
            fx_az -= cxt.model_.hover_accel_z();
          }
        }

        // Clamp acceleration
        fx_ax = clamp(fx_ax, MAX_PREDICTED_ACCEL_XYZ);
        fx_ay = clamp(fx_ay, MAX_PREDICTED_ACCEL_XYZ);
        fx_az = clamp(fx_az, MAX_PREDICTED_ACCEL_XYZ);
        fx_ayaw = clamp(fx_ayaw, MAX_PREDICTED_ACCEL_RPY);

        // Velocity, vx += ax * dt
        fx_vx += fx_ax * dt;
        fx_vy += fx_ay * dt;
        fx_vz += fx_az * dt;
        fx_vyaw += fx_ayaw * dt;

        // Clamp velocity
        fx_vx = clamp(fx_vx, MAX_PREDICTED_VELO_XYZ);
        fx_vy = clamp(fx_vy, MAX_PREDICTED_VELO_XYZ);
        fx_vz = clamp(fx_vz, MAX_PREDICTED_VELO_XYZ);
        fx_vyaw = clamp(fx_vyaw, MAX_PREDICTED_VELO_RPY);

        // Position, x += vx * dt
        fx_x += fx_vx * dt;
        fx_y += fx_vy * dt;
        fx_z += fx_vz * dt;
        fx_yaw = norm_angle(fx_yaw + fx_vyaw * dt);
      });

    // Custom residual and mean functions
    filter_.set_r_x_fn(four_state_residual);
    filter_.set_mean_x_fn(four_state_mean);
  }

  void FourFilter::odom_from_filter(nav_msgs::msg::Odometry &filtered_odom)
  {
    pose_from_fx(filter_.x(), filtered_odom.pose.pose);
    twist_from_fx(filter_.x(), filtered_odom.twist.twist);
    flatten_4x4_covar(filter_.P(), filtered_odom.pose.covariance, 0);
    flatten_4x4_covar(filter_.P(), filtered_odom.twist.covariance, 6);
  }

  void FourFilter::queue_depth(const orca_msgs::msg::Depth &depth)
  {
    rclcpp::Time stamp{depth.header.stamp};

    if (stamp >= filter_time_) {
      RCLCPP_DEBUG(logger_, "queue depth %s", to_str(stamp).c_str());
      Measurement m;
      m.init_z(depth, [](const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> z)
      {
        z(0) = fx_z;
      });
      q_.push(m);
    } else {
      RCLCPP_WARN(logger_, "depth stamp %s is older than filter time %s, dropping", to_str(stamp).c_str(),
                  to_str(filter_time_).c_str());
    }
  }

  void FourFilter::queue_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose)
  {
    rclcpp::Time stamp{pose.header.stamp};

    if (stamp >= filter_time_) {
      RCLCPP_DEBUG(logger_, "queue pose %s", to_str(stamp).c_str());
      Measurement m;
      m.init_4dof(pose, [](const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> z)
      {
        z(0) = fx_x;
        z(1) = fx_y;
        z(2) = fx_z;
        z(3) = fx_yaw;
      });
      q_.push(m);
    } else {
      RCLCPP_WARN(logger_, "pose stamp %s is older than filter time %s, dropping", to_str(stamp).c_str(),
                  to_str(filter_time_).c_str());
    }
  }

} // namespace orca_base
