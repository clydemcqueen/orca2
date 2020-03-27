#include "orca_filter/filter_base.hpp"

#include "eigen3/Eigen/Dense"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_shared/util.hpp"

using namespace orca;

namespace orca_filter
{

  //==================================================================
  // DepthFilter
  //==================================================================

  constexpr int DEPTH_STATE_DIM = 3;      // [z, vz, az]T

  // Depth state macros
#define dx_z x(0)
#define dx_vz x(1)
#define dx_az x(2)

  // Init x from pose
  Eigen::VectorXd pose_to_dx(const geometry_msgs::msg::Pose &pose)
  {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(DEPTH_STATE_DIM);

    dx_z = pose.position.z;

    return x;
  }

  // Extract pose from DepthFilter state
  void pose_from_dx(const Eigen::VectorXd &x, geometry_msgs::msg::Pose &out)
  {
    out.position.x = 0;
    out.position.y = 0;
    out.position.z = dx_z;

    out.orientation.x = 0;
    out.orientation.y = 0;
    out.orientation.z = 0;
    out.orientation.w = 1;
  }

  // Extract twist from PoseFilter state
  void twist_from_dx(const Eigen::VectorXd &x, geometry_msgs::msg::Twist &out)
  {
    out.linear.z = dx_vz;
  }

  // Extract pose or twist covariance from DepthFilter covariance
  void flatten_1x1_covar(const Eigen::MatrixXd &P, std::array<double, 36> &pose_covar, bool pose)
  {
    // Start with extremely high covariance values
    Eigen::MatrixXd m = Eigen::MatrixXd::Identity(6, 6) * 1e6;

    // Copy the z value
    int offset = pose ? 0 : 1;
    m(2, 2) = P(offset, offset);

    // Flatten the 6x6 matrix
    flatten_6x6_covar(m, pose_covar, 0);
  }

  DepthFilter::DepthFilter(const rclcpp::Logger &logger,
                           const FilterContext &cxt,
                           rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr filtered_odom_pub,
                           rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub) :
    FilterBase{Type::depth, logger, cxt, filtered_odom_pub, tf_pub, DEPTH_STATE_DIM}
  {
    filter_.set_Q(Eigen::MatrixXd::Identity(DEPTH_STATE_DIM, DEPTH_STATE_DIM) * 0.01);

    // State transition function
    filter_.set_f_fn(
      [&cxt](const double dt, const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> x)
      {
        if (cxt.predict_accel_) {
          // Assume 0 acceleration
          dx_az = 0;

          if (cxt.predict_accel_control_) {
            // Add acceleration due to control
            dx_az += u(2, 0);
          }

          if (cxt.predict_accel_drag_) {
            // Add acceleration due to drag
            // TODO create & use AddLinkForce(drag_force, c_of_mass) and AddRelativeTorque(drag_torque)
            // Simple approximation:
            dx_az += cxt.model_.drag_accel_z(dx_vz);
          }

          if (cxt.predict_accel_buoyancy_) {
            // Add acceleration due to gravity and buoyancy
            // TODO create & use AddLinkForce(buoyancy_force, c_of_volume)
            // Simple approximation:
            dx_az -= cxt.model_.hover_accel_z();
          }
        }

        // Clamp acceleration
        dx_az = clamp(dx_az, MAX_PREDICTED_ACCEL_XYZ);

        // Velocity, vx += ax * dt
        dx_vz += dx_az * dt;

        // Clamp velocity
        dx_vz = clamp(dx_vz, MAX_PREDICTED_VELO_XYZ);

        // Position, x += vx * dt
        dx_z += dx_vz * dt;
      });
  }

  void DepthFilter::reset(const geometry_msgs::msg::Pose &pose)
  {
    FilterBase::reset(pose_to_dx(pose));
  }

  void DepthFilter::odom_from_filter(orca_msgs::msg::FiducialPoseStamped &filtered_odom)
  {
    pose_from_dx(filter_.x(), filtered_odom.fp.pose.pose);
    // twist_from_dx(filter_.x(), filtered_odom.twist.twist);
    flatten_1x1_covar(filter_.P(), filtered_odom.fp.pose.covariance, true);
    // flatten_1x1_covar(filter_.P(), filtered_odom.twist.covariance, false);
  }

  Measurement DepthFilter::to_measurement(const orca_msgs::msg::Depth &depth,
                                          const orca::Observations &observations) const
  {
    Measurement m;
    m.init_z(depth, observations, [](const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> z)
    {
      z(0) = dx_z;
    });
    return m;
  }

  Measurement DepthFilter::to_measurement(const geometry_msgs::msg::PoseWithCovarianceStamped &pose,
                                          const orca::Observations &observations) const
  {
    // Not supported
    assert(false);
  }

} // namespace orca_filter
