#ifndef ORCA_BASE_FILTER_H
#define ORCA_BASE_FILTER_H

#include <queue>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ukf/ukf.hpp"

#include "orca_msgs/msg/depth.hpp"

#include "orca_base/filter_context.hpp"
#include "orca_base/geometry.hpp"

namespace orca_base
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

    // Must be default constructible
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
  // Filters
  //=============================================================================

  class FilterBase
  {
    const rclcpp::Duration LAG{RCL_MS_TO_NS(50)};
    const rclcpp::Duration TOO_OLD{RCL_MS_TO_NS(250)};

    void predict(const rclcpp::Time &stamp, const Acceleration &u_bar);

  protected:

    rclcpp::Logger logger_;
    const FilterContext &cxt_;

    // Filter
    int state_dim_;
    ukf::UnscentedKalmanFilter filter_;
    rclcpp::Time filter_time_{0, 0, RCL_ROS_TIME};

    // Measurement queue
    std::priority_queue<Measurement, std::vector<Measurement>, Measurement> q_;

    virtual void odom_from_filter(nav_msgs::msg::Odometry &filtered_odom) = 0;

  public:

    explicit FilterBase(const rclcpp::Logger &logger, const FilterContext &cxt_, int state_dim);

    void reset();

    bool filter_valid()
    { return filter_.valid(); }

    virtual void queue_depth(const orca_msgs::msg::Depth &depth)
    { assert(false); }

    virtual void queue_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose)
    { assert(false); }

    // Run the filter, return true if there's a odometry message to publish
    bool process(const rclcpp::Time &t, const Acceleration &u_bar, nav_msgs::msg::Odometry &filtered_odom);
  };

  // Filter only z (depth)
  class DepthFilter : public FilterBase
  {
    void odom_from_filter(nav_msgs::msg::Odometry &filtered_odom) override;

  public:

    explicit DepthFilter(const rclcpp::Logger &logger, const FilterContext &cxt_);

    void queue_depth(const orca_msgs::msg::Depth &depth) override;
  };

  // Filter 4 DoF, assume roll and pitch are always 0
  class FourFilter : public FilterBase
  {
    void odom_from_filter(nav_msgs::msg::Odometry &filtered_odom) override;

  public:

    explicit FourFilter(const rclcpp::Logger &logger, const FilterContext &cxt_);

    void queue_depth(const orca_msgs::msg::Depth &depth) override;

    void queue_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose) override;
  };

  // Filter all 6 DoF
  class PoseFilter : public FilterBase
  {
    void odom_from_filter(nav_msgs::msg::Odometry &filtered_odom) override;

  public:

    explicit PoseFilter(const rclcpp::Logger &logger, const FilterContext &cxt_);

    void queue_depth(const orca_msgs::msg::Depth &depth) override ;

    void queue_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose) override ;
  };

} // namespace orca_base

#endif // ORCA_BASE_FILTER_H
