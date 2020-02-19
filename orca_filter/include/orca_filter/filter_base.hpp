#ifndef ORCA_FILTER_FILTER_H
#define ORCA_FILTER_FILTER_H

#include <queue>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logger.hpp"
#include "ukf/ukf.hpp"

#include "orca_msgs/msg/depth.hpp"
#include "orca_shared/geometry.hpp"

#include "orca_filter/filter_context.hpp"

namespace orca_filter
{

  //=============================================================================
  // Constants
  //=============================================================================

  // Maximum predicted acceleration
  // The AUV may be tossed around by waves or bump into something
  constexpr double MAX_PREDICTED_ACCEL_XYZ = 100;
  constexpr double MAX_PREDICTED_ACCEL_RPY = 100;

  // Maximum predicted velocity in water
  constexpr double MAX_PREDICTED_VELO_XYZ = 100;
  constexpr double MAX_PREDICTED_VELO_RPY = 100;

  //==================================================================
  // Unscented residual and mean functions for PoseFilter 6dof state (x) and 6dof pose measurement (z)
  //
  // Because of the layout of the state and pose measurement matrices and the way that Eigen works
  // these functions can serve double-duty.
  //
  // 6d x:       [x, y, z, r, p, y, ...]
  // 6d pose z:  [x, y, z, r, p, y]
  //
  // The mean function needs to compute the mean of angles, which doesn't have a precise meaning.
  // See https://en.wikipedia.org/wiki/Mean_of_circular_quantities for the method used here.
  //
  // There are similar residual and mean functions for FourFilter 4dof state (x) and 4dof pose measurement (z)
  //==================================================================

  Eigen::VectorXd six_state_residual(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::VectorXd &mean);

  Eigen::VectorXd six_state_mean(const Eigen::MatrixXd &sigma_points, const Eigen::RowVectorXd &Wm);

  Eigen::VectorXd four_state_residual(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::VectorXd &mean);

  Eigen::VectorXd four_state_mean(const Eigen::MatrixXd &sigma_points, const Eigen::RowVectorXd &Wm);

  //=============================================================================
  // Utility for 6dof covariance matrices
  //=============================================================================

  void flatten_6x6_covar(const Eigen::MatrixXd &m, std::array<double, 36> &covar, int offset);

  //=============================================================================
  // Measurements
  //=============================================================================

  struct Measurement
  {
    rclcpp::Time stamp_;
    Eigen::VectorXd z_;
    Eigen::MatrixXd R_;
    ukf::MeasurementFn h_fn_;
    ukf::ResidualFn r_z_fn_;
    ukf::UnscentedMeanFn mean_z_fn_;

    // Must be default constructible to be used in a priority queue
    Measurement() = default;

    // 1dof z measurement from a depth message
    void init_z(const orca_msgs::msg::Depth &depth, ukf::MeasurementFn h_fn);

    // 4dof measurement from a pose message
    void init_4dof(const geometry_msgs::msg::PoseWithCovarianceStamped &pose, ukf::MeasurementFn h_fn);

    // 6dof measurement from a pose message
    void init_6dof(const geometry_msgs::msg::PoseWithCovarianceStamped &pose, ukf::MeasurementFn h_fn);

    // Sort by time
    bool operator()(const Measurement &a, const Measurement &b)
    {
      return a.stamp_ > b.stamp_;
    }
  };

  //=============================================================================
  // Filter state
  //=============================================================================

  struct State
  {
    rclcpp::Time stamp_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;

    // Must be default constructible
    State() = default;

    State(const rclcpp::Time &stamp, Eigen::VectorXd x, Eigen::MatrixXd P) :
      stamp_{stamp}, x_{std::move(x)}, P_{std::move(P)}
    {}
  };

  //=============================================================================
  // Filter base
  //=============================================================================

  class FilterBase
  {
    const rclcpp::Duration HISTORY_LENGTH{RCL_S_TO_NS(1)};

    int state_dim_;

    // Current time of filter
    rclcpp::Time filter_time_;

    // Measurement priority queue, sorted from oldest to newest
    std::priority_queue<Measurement, std::vector<Measurement>, Measurement> measurement_q_;

    // State history, ordered from oldest to newest
    std::deque<State> state_history_;

    // Measurement history, ordered from oldest to newest
    std::deque<Measurement> measurement_history_;

    // Call filter_->predict
    void predict(const rclcpp::Time &stamp, const orca::Acceleration &u_bar);

    // Process all messages in the queue, return true if there's an odometry message to publish
    bool process(const rclcpp::Time &stamp, const orca::Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom);

    // Rewind to a previous state
    bool rewind(const rclcpp::Time &stamp);

  protected:

    rclcpp::Logger logger_;
    const FilterContext &cxt_;

    ukf::UnscentedKalmanFilter filter_;

    // Reset the filter with an Eigen vector
    void reset(const Eigen::VectorXd &x);

    virtual void odom_from_filter(nav_msgs::msg::Odometry &filtered_odom) = 0;

    // Convert a Depth message to a Measurement
    virtual Measurement to_measurement(const orca_msgs::msg::Depth &depth) const = 0;

    // Convert PoseWithCovarianceStamped message to a Measurement
    virtual Measurement to_measurement(const geometry_msgs::msg::PoseWithCovarianceStamped &pose) const = 0;

  public:

    explicit FilterBase(const rclcpp::Logger &logger, const FilterContext &cxt_, int state_dim);

    // Reset the filter
    void reset();

    // Reset the filter with a pose
    virtual void reset(const geometry_msgs::msg::Pose &pose) = 0;

    // Is the filter valid?
    bool filter_valid()
    { return filter_.valid(); }

    // Process a message
    template<typename T>
    bool process_message(const T &msg, const orca::Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom)
    {
      rclcpp::Time stamp{msg.header.stamp};

      if (stamp < filter_time_ && !rewind(stamp)) {
        // This message is out of order, and we can't rewind history
        return false;
      }

      // Add this message to the priority queue
      measurement_q_.push(to_measurement(msg));

      // Process one or more measurements
      return process(stamp, u_bar, filtered_odom);
    }
  };

  //=============================================================================
  // Filter only z (depth)
  //=============================================================================

  class DepthFilter : public FilterBase
  {
    void odom_from_filter(nav_msgs::msg::Odometry &filtered_odom) override;

    Measurement to_measurement(const orca_msgs::msg::Depth &depth) const override;

    Measurement to_measurement(const geometry_msgs::msg::PoseWithCovarianceStamped &pose) const override;

  public:

    explicit DepthFilter(const rclcpp::Logger &logger, const FilterContext &cxt_);

    // Reset the filter with a pose
    void reset(const geometry_msgs::msg::Pose &pose) override;
  };

  //=============================================================================
  // Filter 4 DoF, assume roll and pitch are always 0
  //=============================================================================

  class FourFilter : public FilterBase
  {
    void odom_from_filter(nav_msgs::msg::Odometry &filtered_odom) override;

    Measurement to_measurement(const orca_msgs::msg::Depth &depth) const override;

    Measurement to_measurement(const geometry_msgs::msg::PoseWithCovarianceStamped &pose) const override;

  public:

    explicit FourFilter(const rclcpp::Logger &logger, const FilterContext &cxt);

    // Reset the filter with a pose
    void reset(const geometry_msgs::msg::Pose &pose) override;
  };

  //=============================================================================
  // Filter all 6 DoF
  //=============================================================================

  class PoseFilter : public FilterBase
  {
    void odom_from_filter(nav_msgs::msg::Odometry &filtered_odom) override;

  public:

    explicit PoseFilter(const rclcpp::Logger &logger, const FilterContext &cxt);

    // Reset the filter with a pose
    void reset(const geometry_msgs::msg::Pose &pose) override;

    Measurement to_measurement(const orca_msgs::msg::Depth &depth) const override;

    Measurement to_measurement(const geometry_msgs::msg::PoseWithCovarianceStamped &pose) const override;
  };

} // namespace orca_filter

#endif // ORCA_FILTER_FILTER_H
