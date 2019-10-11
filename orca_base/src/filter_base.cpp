#include "orca_base/filter_base.hpp"

#include "eigen3/Eigen/Dense"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_base/model.hpp"
#include "orca_base/util.hpp"

namespace orca_base
{

  //=============================================================================
  // Constants
  //=============================================================================

  constexpr int CONTROL_DIM = 4;          // [ax, ay, az, ayaw]T

  constexpr double MIN_DT = 0.001;
  constexpr double DEFAULT_DT = 0.1;
  constexpr double MAX_DT = 1.0;

  //==================================================================
  // Utility functions
  //==================================================================

  bool valid_stamp(const rclcpp::Time &stamp)
  {
    return stamp.nanoseconds() > 0;
  }

  // Create control matrix u
  void to_u(const Acceleration &in, Eigen::VectorXd &out)
  {
    out = Eigen::VectorXd(CONTROL_DIM);
    out << in.x, in.y, in.z, in.yaw;
  }

  // Flatten a 6x6 covariance matrix
  void flatten_6x6_covar(const Eigen::MatrixXd &m, std::array<double, 36> &covar, int offset)
  {
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        covar[i * 6 + j] = m(i + offset, j + offset);
      }
    }
  }

  //==================================================================
  // Measurement
  //==================================================================

  void Measurement::init_z(const orca_msgs::msg::Depth &depth, ukf::MeasurementFn h_fn)
  {
    stamp_ = depth.header.stamp;

    z_ = Eigen::VectorXd(1);
    z_ << depth.z;

    R_ = Eigen::MatrixXd(1, 1);
    R_ << depth.z_variance;

    h_fn_ = std::move(h_fn);

    // Use the standard residual and mean functions for depth measurements
    r_z_fn_ = ukf::residual;
    mean_z_fn_ = ukf::unscented_mean;
  }

  void Measurement::init_4dof(const geometry_msgs::msg::PoseWithCovarianceStamped &pose, ukf::MeasurementFn h_fn)
  {
    stamp_ = pose.header.stamp;

    tf2::Transform t_map_base;
    tf2::fromMsg(pose.pose.pose, t_map_base);

    tf2Scalar roll, pitch, yaw;
    t_map_base.getBasis().getRPY(roll, pitch, yaw);

    z_ = Eigen::VectorXd(4);
    z_ << t_map_base.getOrigin().x(), t_map_base.getOrigin().y(), t_map_base.getOrigin().z(), yaw;

    R_ = Eigen::MatrixXd(4, 4);
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        // Copy rows {0, 1, 2, 5} and cols {0, 1, 2, 5}
        R_(i, j) = pose.pose.covariance[(i < 3 ? i : 5) * 6 + (j < 3 ? j : 5)];
      }
    }

    h_fn_ = std::move(h_fn);

    // Use a custom residual and mean functions for pose measurements
    r_z_fn_ = four_state_residual;
    mean_z_fn_ = four_state_mean;
  }

  void Measurement::init_6dof(const geometry_msgs::msg::PoseWithCovarianceStamped &pose, ukf::MeasurementFn h_fn)
  {
    stamp_ = pose.header.stamp;

    tf2::Transform t_map_base;
    tf2::fromMsg(pose.pose.pose, t_map_base);

    tf2Scalar roll, pitch, yaw;
    t_map_base.getBasis().getRPY(roll, pitch, yaw);

    z_ = Eigen::VectorXd(6);
    z_ << t_map_base.getOrigin().x(), t_map_base.getOrigin().y(), t_map_base.getOrigin().z(), roll, pitch, yaw;

    R_ = Eigen::MatrixXd(6, 6);
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        R_(i, j) = pose.pose.covariance[i * 6 + j];
      }
    }

    h_fn_ = std::move(h_fn);

    // Use a custom residual and mean functions for pose measurements
    r_z_fn_ = six_state_residual;
    mean_z_fn_ = six_state_mean;
  }

  //==================================================================
  // FilterBase
  //==================================================================

  FilterBase::FilterBase(const rclcpp::Logger &logger, const FilterContext &cxt, int state_dim) :
    logger_{logger},
    cxt_{cxt},
    state_dim_{state_dim},
    filter_{state_dim, 0.001, 2.0, 0}
  {
    reset();
  }

  void FilterBase::reset()
  {
    filter_.set_x(Eigen::VectorXd::Zero(state_dim_));
    filter_.set_P(Eigen::MatrixXd::Identity(state_dim_, state_dim_)); // Big P! The first measurement will set x
  }

  void FilterBase::predict(const rclcpp::Time &stamp, const Acceleration &u_bar)
  {
    // Filter time starts at 0, test for this
    if (!valid_stamp(filter_time_)) {
      RCLCPP_INFO(logger_, "start filter, stamp %s", to_str(stamp).c_str());
    } else {
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
        RCLCPP_DEBUG(logger_, "skip predict, stamp %s, filter %s",
                     to_str(stamp).c_str(), to_str(filter_time_).c_str());
      } else {
        // Run the prediction
        RCLCPP_DEBUG(logger_, "predict, stamp %s, filter %s",
                     to_str(stamp).c_str(), to_str(filter_time_).c_str());
        Eigen::VectorXd u;
        to_u(u_bar, u);
        filter_.predict(dt, u);
      }
    }

    // Always advance filter_time_
    filter_time_ = stamp;
  }

  bool FilterBase::process(const rclcpp::Time &t, const Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom)
  {
    rclcpp::Time too_old = t - TOO_OLD;
    rclcpp::Time start = filter_time_ > too_old ? filter_time_ : too_old;
    rclcpp::Time end = t - LAG;

    // Pop measurements older than start
    while (!q_.empty() && q_.top().stamp_ < start) {
      RCLCPP_WARN(logger_, "measurement %s is older than %s, dropping",
                  to_str(q_.top().stamp_).c_str(), to_str(start).c_str());

      q_.pop();
    }

    // If there are no measurements return false
    if (q_.empty() || q_.top().stamp_ >= end) {
      RCLCPP_DEBUG(logger_, "no measurements to process");
      return false;
    }

    // Set outlier distance
    filter_.set_outlier_distance(cxt_.outlier_distance_);

    // Keep track of inliers and outliers
    int inliers = 0;
    int outliers = 0;

    // Get measurements between start and end
    while (!q_.empty() && q_.top().stamp_ < end) {
      RCLCPP_DEBUG(logger_, "measurement %s is older than %s, processing",
                   to_str(q_.top().stamp_).c_str(), to_str(end).c_str());

      Measurement m = q_.top();
      q_.pop();

      predict(m.stamp_, u_bar);

      filter_.set_h_fn(m.h_fn_);
      filter_.set_r_z_fn(m.r_z_fn_);
      filter_.set_mean_z_fn(m.mean_z_fn_);

      if (filter_.update(m.z_, m.R_)) {
        inliers++;
      } else {
        outliers++;
      }
    }

    if (outliers) {
      RCLCPP_INFO(logger_, "rejected %d outlier(s) at %s", outliers, to_str(end).c_str());
    }

    if (!inliers || !filter_.valid()) {
      return false;
    }

    // Return a new estimate
    filtered_odom.header.stamp = filter_time_;
    odom_from_filter(filtered_odom);

    return true;
  }

} // namespace orca_base
