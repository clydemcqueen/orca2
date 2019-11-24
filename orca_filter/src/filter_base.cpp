#include "orca_filter/filter_base.hpp"

#include "eigen3/Eigen/Dense"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "orca_shared/util.hpp"

using namespace orca;

namespace orca_filter
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
    reset(Eigen::VectorXd::Zero(state_dim_));
  }

  void FilterBase::reset(const Eigen::VectorXd &x)
  {
    // Clear all pending measurements
    measurement_q_ = std::priority_queue<Measurement, std::vector<Measurement>, Measurement>();

    // Clear history
    state_history_.clear();
    measurement_history_.clear();

    // Reset filter time
    filter_time_ = {0, 0, RCL_ROS_TIME};

    // Start with a default state and a large covariance matrix
    filter_.set_x(x);
    filter_.set_P(Eigen::MatrixXd::Identity(state_dim_, state_dim_));
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

  bool FilterBase::process(const rclcpp::Time &stamp, const Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom)
  {
    // Trim state_history_
    while (!state_history_.empty() && state_history_.front().stamp_ < stamp - HISTORY_LENGTH) {
      RCLCPP_DEBUG(logger_, "pop old state history %s", to_str(state_history_.front().stamp_).c_str());
      state_history_.pop_front();
    }

    // Trim measurement_history_
    while (!measurement_history_.empty() && measurement_history_.front().stamp_ < stamp - HISTORY_LENGTH) {
      RCLCPP_DEBUG(logger_, "pop old measurement history %s", to_str(measurement_history_.front().stamp_).c_str());
      measurement_history_.pop_front();
    }

    // Set outlier distance, by doing this each time we can change this on-the-fly
    filter_.set_outlier_distance(cxt_.outlier_distance_);

    // Keep track of inliers and outliers
    int inliers = 0;
    int outliers = 0;

    // Process all measurements
    while (!measurement_q_.empty()) {
      RCLCPP_DEBUG(logger_, "processing measurement %s", to_str(measurement_q_.top().stamp_).c_str());

      Measurement m = measurement_q_.top();
      measurement_q_.pop();

      predict(m.stamp_, u_bar);

      filter_.set_h_fn(m.h_fn_);
      filter_.set_r_z_fn(m.r_z_fn_);
      filter_.set_mean_z_fn(m.mean_z_fn_);

      if (filter_.update(m.z_, m.R_)) {
        inliers++;
      } else {
        outliers++;
      }

      // Save measurement in history
      measurement_history_.push_back(m);

      // Save state in history
      state_history_.emplace_back(m.stamp_, filter_.x(), filter_.P());
    }

    if (outliers) {
      RCLCPP_DEBUG(logger_, "rejected %d outlier(s)", outliers);
    }

    if (!inliers || !filter_.valid()) {
      return false;
    }

    // Return a new estimate
    filtered_odom.header.stamp = filter_time_;
    odom_from_filter(filtered_odom);

    return true;
  }

  // Rewind to a previous state, return true if successful, false if there was no change
  bool FilterBase::rewind(const rclcpp::Time &stamp)
  {
    if (state_history_.empty() || stamp < state_history_.front().stamp_) {
      RCLCPP_WARN(logger_, "can't rewind to %s, dropping message", to_str(stamp).c_str());
      return false;
    }

    // Pop newer states
    while (!state_history_.empty() && state_history_.back().stamp_ > stamp) {
      RCLCPP_DEBUG(logger_, "rewind: pop state %s", to_str(stamp).c_str());
      state_history_.pop_back();
    }

    // Set the filter state
    RCLCPP_DEBUG(logger_, "rewind %dms", (stamp - state_history_.back().stamp_).nanoseconds() / 1000000);
    filter_time_ = state_history_.back().stamp_;
    filter_.set_x(state_history_.back().x_);
    filter_.set_P(state_history_.back().P_);

    // Pop newer measurements and put them back into the priority queue
    while (!measurement_history_.empty() && measurement_history_.back().stamp_ > stamp) {
      RCLCPP_DEBUG(logger_, "rewind: re-queue measurement %s", to_str(stamp).c_str());
      measurement_q_.push(measurement_history_.back());
      measurement_history_.pop_back();
    }

    return true;
  }

} // namespace orca_filter
