#include "orca_base/filter_base.hpp"

#include "eigen3/Eigen/Dense"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_base/model.hpp"
#include "orca_base/util.hpp"

namespace orca_base
{

  //==================================================================
  // DepthFilter
  //==================================================================

  constexpr int DEPTH_STATE_DIM = 3;      // [z, vz, az]T

  // Depth state macros
#define dx_z x(0)
#define dx_vz x(1)
#define dx_az x(2)

  DepthFilter::DepthFilter(const rclcpp::Logger &logger, const FilterContext &cxt) :
    FilterBase{logger, cxt, DEPTH_STATE_DIM}
  {
    filter_.set_Q(Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 0.01);

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

  void DepthFilter::odom_from_filter(nav_msgs::msg::Odometry &filtered_odom)
  {
    // TODO get z, vz and variance
    // Everything else is made up
//    pose_from_x(filter_.x(), filtered_odom.pose.pose);
//    twist_from_x(filter_.x(), filtered_odom.twist.twist);
//    pose_covar_from_P(filter_.P(), filtered_odom.pose.covariance);
//    twist_covar_from_P(filter_.P(), filtered_odom.twist.covariance);
  }

  void DepthFilter::queue_depth(const orca_msgs::msg::Depth &depth)
  {
    rclcpp::Time stamp{depth.header.stamp};

    if (stamp >= filter_time_) {
      RCLCPP_DEBUG(logger_, "queue depth %s", to_str(stamp).c_str());
      Measurement m;
      m.init_z(depth, [](const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> z)
      {
        z(0) = dx_z;
      });
      q_.push(m);
    } else {
      RCLCPP_WARN(logger_, "depth stamp %s is older than filter time %s, dropping", to_str(stamp).c_str(),
                  to_str(filter_time_).c_str());
    }
  }

} // namespace orca_base
