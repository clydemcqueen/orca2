#include "orca_filter/filter_base.hpp"

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/twist.hpp"
#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_filter
{

  //==================================================================
  // PoseFilter -- 6dof filter with 18 dimensions
  //==================================================================

  constexpr int POSE_STATE_DIM = 18;      // [x, y, ..., vx, vy, ..., ax, ay, ...]T

  // Pose state macros
#define px_x x(0)
#define px_y x(1)
#define px_z x(2)
#define px_roll x(3)
#define px_pitch x(4)
#define px_yaw x(5)
#define px_vx x(6)
#define px_vy x(7)
#define px_vz x(8)
#define px_vroll x(9)
#define px_vpitch x(10)
#define px_vyaw x(11)
#define px_ax x(12)
#define px_ay x(13)
#define px_az x(14)
#define px_aroll x(15)
#define px_apitch x(16)
#define px_ayaw x(17)

  // Init x from pose
  Eigen::VectorXd pose_to_px(const geometry_msgs::msg::Pose &pose)
  {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(POSE_STATE_DIM);

    px_x = pose.position.x;
    px_y = pose.position.y;
    px_z = pose.position.z;

    mw::Quaternion q{pose.orientation};
    px_roll = q.roll();
    px_pitch = q.pitch();
    px_yaw = q.yaw();

    return x;
  }

  // Extract pose from PoseFilter state
  void pose_from_px(const Eigen::VectorXd &x, geometry_msgs::msg::Pose &out)
  {
    out.position.x = px_x;
    out.position.y = px_y;
    out.position.z = px_z;

    tf2::Matrix3x3 m;
    m.setRPY(px_roll, px_pitch, px_yaw);

    tf2::Quaternion q;
    m.getRotation(q);

    out.orientation = tf2::toMsg(q);
  }

  // Extract twist from PoseFilter state
  void twist_from_px(const Eigen::VectorXd &x, geometry_msgs::msg::Twist &out)
  {
    out.linear.x = px_vx;
    out.linear.y = px_vy;
    out.linear.z = px_vz;

    out.angular.x = px_vroll;
    out.angular.y = px_vpitch;
    out.angular.z = px_vyaw;
  }

  // Extract pose covariance from PoseFilter covariance
  void pose_covar_from_pP(const Eigen::MatrixXd &P, std::array<double, 36> &pose_covar)
  {
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        pose_covar[i * 6 + j] = P(i, j);  // [0:5, 0:5]
      }
    }
  }

  // Extract twist covariance from PoseFilter covariance
  void twist_covar_from_pP(const Eigen::MatrixXd &P, std::array<double, 36> &twist_covar)
  {
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        twist_covar[i * 6 + j] = P(i + 6, j + 6);  // [6:12, 6:12]
      }
    }
  }

  Eigen::VectorXd six_state_residual(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::VectorXd &mean)
  {
    // Residual for all fields
    Eigen::VectorXd residual = x - mean;

    // Normalize roll, pitch and yaw
    residual(3) = orca::norm_angle(residual(3));
    residual(4) = orca::norm_angle(residual(4));
    residual(5) = orca::norm_angle(residual(5));

    return residual;
  }

  Eigen::VectorXd six_state_mean(const Eigen::MatrixXd &sigma_points, const Eigen::RowVectorXd &Wm)
  {
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(sigma_points.rows());

    // Standard mean for all fields
    for (long i = 0; i < sigma_points.cols(); ++i) {
      mean += Wm(i) * sigma_points.col(i);
    }

    // Sum the sines and cosines
    double sum_r_sin = 0.0, sum_r_cos = 0.0;
    double sum_p_sin = 0.0, sum_p_cos = 0.0;
    double sum_y_sin = 0.0, sum_y_cos = 0.0;

    for (long i = 0; i < sigma_points.cols(); ++i) {
      sum_r_sin += Wm(i) * sin(sigma_points(3, i));
      sum_r_cos += Wm(i) * cos(sigma_points(3, i));

      sum_p_sin += Wm(i) * sin(sigma_points(4, i));
      sum_p_cos += Wm(i) * cos(sigma_points(4, i));

      sum_y_sin += Wm(i) * sin(sigma_points(5, i));
      sum_y_cos += Wm(i) * cos(sigma_points(5, i));
    }

    // Mean is arctan2 of the sums
    mean(3) = atan2(sum_r_sin, sum_r_cos);
    mean(4) = atan2(sum_p_sin, sum_p_cos);
    mean(5) = atan2(sum_y_sin, sum_y_cos);

    return mean;
  }

  PoseFilter::PoseFilter(const rclcpp::Logger &logger,
                         const FilterContext &cxt,
                         rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped2>::SharedPtr filtered_odom_pub,
                         rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub) :
    FilterBase{Type::pose, logger, cxt, filtered_odom_pub, tf_pub, POSE_STATE_DIM}
  {
    filter_.set_Q(Eigen::MatrixXd::Identity(POSE_STATE_DIM, POSE_STATE_DIM) * 0.01);

    // State transition function
    filter_.set_f_fn(
      [&cxt](const double dt, const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> x)
      {
        if (cxt.predict_accel_) {
          // Assume 0 acceleration
          px_ax = 0;
          px_ay = 0;
          px_az = 0;
          px_aroll = 0;
          px_apitch = 0;
          px_ayaw = 0;

          if (cxt.predict_accel_control_) {
            // Add acceleration due to control
            px_ax += u(0, 0);
            px_ay += u(1, 0);
            px_az += u(2, 0);
            px_ayaw += u(3, 0);
          }

          if (cxt.predict_accel_drag_) {
            // Add acceleration due to drag
            // TODO create & use AddLinkForce(drag_force, c_of_mass) and AddRelativeTorque(drag_torque)
            // Simple approximation:
            px_ax += cxt.model_.drag_accel_f(px_vx);  // TODO f or x?
            px_ay += cxt.model_.drag_accel_s(px_vy);  // TODO s or y?
            px_az += cxt.model_.drag_accel_z(px_vz);
            px_aroll += cxt.model_.drag_accel_yaw(px_vroll);
            px_apitch += cxt.model_.drag_accel_yaw(px_vpitch);
            px_ayaw += cxt.model_.drag_accel_yaw(px_vyaw);
          }

          if (cxt.predict_accel_buoyancy_) {
            // Add acceleration due to gravity and buoyancy
            // TODO create & use AddLinkForce(buoyancy_force, c_of_volume)
            // Simple approximation:
            px_roll = 0;
            px_pitch = 0;
            px_az -= cxt.model_.hover_accel_z();
          }
        }

        // Clamp acceleration
        px_ax = orca::clamp(px_ax, MAX_PREDICTED_ACCEL_XYZ);
        px_ay = orca::clamp(px_ay, MAX_PREDICTED_ACCEL_XYZ);
        px_az = orca::clamp(px_az, MAX_PREDICTED_ACCEL_XYZ);
        px_aroll = orca::clamp(px_aroll, MAX_PREDICTED_ACCEL_RPY);
        px_apitch = orca::clamp(px_apitch, MAX_PREDICTED_ACCEL_RPY);
        px_ayaw = orca::clamp(px_ayaw, MAX_PREDICTED_ACCEL_RPY);

        // Velocity, vx += ax * dt
        px_vx += px_ax * dt;
        px_vy += px_ay * dt;
        px_vz += px_az * dt;
        px_vroll += px_aroll * dt;
        px_vpitch += px_apitch * dt;
        px_vyaw += px_ayaw * dt;

        // Clamp velocity
        px_vx = orca::clamp(px_vx, MAX_PREDICTED_VELO_XYZ);
        px_vy = orca::clamp(px_vy, MAX_PREDICTED_VELO_XYZ);
        px_vz = orca::clamp(px_vz, MAX_PREDICTED_VELO_XYZ);
        px_vroll = orca::clamp(px_vroll, MAX_PREDICTED_VELO_RPY);
        px_vpitch = orca::clamp(px_vpitch, MAX_PREDICTED_VELO_RPY);
        px_vyaw = orca::clamp(px_vyaw, MAX_PREDICTED_VELO_RPY);

        // Position, x += vx * dt
        px_x += px_vx * dt;
        px_y += px_vy * dt;
        px_z += px_vz * dt;
        px_roll = orca::norm_angle(px_roll + px_vroll * dt);
        px_pitch = orca::norm_angle(px_pitch + px_vpitch * dt);
        px_yaw = orca::norm_angle(px_yaw + px_vyaw * dt);
      });

    // Custom residual and mean functions
    filter_.set_r_x_fn(six_state_residual);
    filter_.set_mean_x_fn(six_state_mean);
  }

  void PoseFilter::reset(const geometry_msgs::msg::Pose &pose)
  {
    FilterBase::reset(pose_to_px(pose));
  }

  void PoseFilter::odom_from_filter(orca_msgs::msg::FiducialPose2 &filtered_odom)
  {
    pose_from_px(filter_.x(), filtered_odom.pose.pose);
    // twist_from_px(filter_.x(), filtered_odom.twist.twist);
    flatten_6x6_covar(filter_.P(), filtered_odom.pose.covariance, 0);
    // flatten_6x6_covar(filter_.P(), filtered_odom.twist.covariance, 6);
  }

  Measurement PoseFilter::to_measurement(const orca_msgs::msg::Depth &depth,
                                         const mw::Observations &observations) const
  {
    Measurement m;
    m.init_z(depth, observations, [](const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> z)
    {
      z(0) = px_z;
    });
    return m;
  }

  Measurement PoseFilter::to_measurement(const geometry_msgs::msg::PoseWithCovarianceStamped &pose,
                                         const mw::Observations &observations) const
  {
    Measurement m;
    m.init_6dof(pose, observations, [](const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> z)
    {
      z(0) = px_x;
      z(1) = px_y;
      z(2) = px_z;
      z(3) = px_roll;
      z(4) = px_pitch;
      z(5) = px_yaw;
    });
    return m;
  }

} // namespace orca_filter
