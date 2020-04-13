#include "orca_filter/filter_node.hpp"

#include "orca_filter/perf.hpp"
#include "orca_shared/util.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/*******************************************************************************************************
 * FilterNode runs one of three filters:
 *
 * PoseFilter filters all 6 DoF
 * FourFilter filters x, y, z and yaw, and sets roll and pitch to 0
 * DepthFilter filters z, all other DoF are set to 0
 *
 * FilterNode can be in one of two states:
 *
 * !good_pose_:      There are no marker observations, or the markers are too far away to be useful
 *                   Use a DepthFilter at the barometer rate -- 20Hz
 *                   Output pose is (0, 0, filtered z, 0, 0, 0)
 *                   Output observations are copied from the last FiducialPoseStamped message received,
 *                   or no observations if the last FiducialPoseStamped message is stale
 *
 * good_pose:        Markers are close enough to generate a good pose
 *                   Use a PoseFilter or a FourFilter at the camera rate -- 30Hz
 *                   FiducialPose observations are passed through unfiltered
 *                   Depth messages are fused, but don't result in published odometry
 */

namespace orca_filter
{

  //=============================================================================
  // FilterNode
  //=============================================================================

  constexpr int QUEUE_SIZE = 10;

  FilterNode::FilterNode() : Node{"filter_node"}
  {
    // Suppress IDE warnings
    (void) depth_sub_;
    (void) control_sub_;
    (void) fcam_sub_;

    // Create this before calling validate_parameters()
    filtered_odom_pub_ = create_publisher<orca_msgs::msg::FiducialPoseStamped>("filtered_fp", QUEUE_SIZE);

    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(FILTER_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), FILTER_NODE_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    FILTER_NODE_ALL_PARAMS

    // Parse URDF
    if (!parser_.parse()) {
      RCLCPP_ERROR(get_logger(), "can't parse URDF %s", orca_description::filename);
    }

    RCLCPP_INFO(get_logger(), "filter_node ready");
  }

  void FilterNode::validate_parameters()
  {
    // Set up additional publications and subscriptions
    if (cxt_.filter_baro_) {
      depth_sub_ = create_subscription<orca_msgs::msg::Depth>(
        "depth", QUEUE_SIZE, [this](const orca_msgs::msg::Depth::SharedPtr msg) -> void
        { this->depth_cb_.call(msg); });
    } else {
      depth_sub_.reset();
    }

    if (cxt_.filter_fcam_) {
      fcam_sub_ = create_subscription<orca_msgs::msg::FiducialPoseStamped>(
        "fcam_fp", QUEUE_SIZE, [this](const orca_msgs::msg::FiducialPoseStamped::SharedPtr msg) -> void
        { this->fcam_cb_.call(msg); });
    } else {
      fcam_sub_.reset();
    }

    if (cxt_.publish_tf_) {
      tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", QUEUE_SIZE);
    } else {
      tf_pub_.reset();
    }

    // Update model from new parameters
    cxt_.model_.fluid_density_ = cxt_.param_fluid_density_;

    // Update timeouts
    open_water_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_open_water_ms_)};
    outlier_timeout_ = rclcpp::Duration{RCL_MS_TO_NS(cxt_.timeout_outlier_ms_)};

    create_filter();
  }

  void FilterNode::create_filter()
  {
    bool good_pose = observations_.closest_distance() < cxt_.good_pose_dist_;

    if (filter_ && good_pose_ == good_pose) {
      // Nothing to do
      return;
    }

    good_pose_ = good_pose;

    if (good_pose_) {
      RCLCPP_INFO(get_logger(), "good pose");
    } else {
      RCLCPP_INFO(get_logger(), "no markers, or markers are too far away for a good pose");
    }

    // Create a filter if required
    if (good_pose_) {
      if (cxt_.four_dof_ && (!filter_ || filter_->type() != FilterBase::Type::four)) {
        filter_ = std::make_shared<FourFilter>(get_logger(), cxt_, filtered_odom_pub_, tf_pub_);
      } else if (!filter_ || filter_->type() != FilterBase::Type::pose) {
        filter_ = std::make_shared<PoseFilter>(get_logger(), cxt_, filtered_odom_pub_, tf_pub_);
      }
    } else if (!filter_ || filter_->type() != FilterBase::Type::depth) {
      filter_ = std::make_shared<DepthFilter>(get_logger(), cxt_, filtered_odom_pub_, tf_pub_);
    }
  }

  // New depth reading
  void FilterNode::depth_callback(const orca_msgs::msg::Depth::SharedPtr msg, bool first)
  {
    START_PERF()

    if (cxt_.filter_baro_) {

      if (!observations_.empty()) {
        rclcpp::Time stamp{msg->header.stamp};
        if (orca::valid_stamp(last_fp_stamp_) && stamp - last_fp_stamp_ > open_water_timeout_) {
          // Timeout... clear observations and create a depth filter
          observations_.clear();
          create_filter();
        }
      }

      filter_->process_message(*msg, observations_);
    }

    STOP_PERF("depth_callback")
  }

  void FilterNode::control_callback(const orca_msgs::msg::Control::SharedPtr msg, bool first)
  {
    // TODO must add force parameters to orca_filter to handle u_bar input
#if 0
    Efforts e;
    e.from_msg(msg->efforts);
    e.to_acceleration(estimated_yaw_, u_bar_);
#endif
  }

  void FilterNode::fcam_callback(const orca_msgs::msg::FiducialPoseStamped::SharedPtr msg, bool first)
  {
    START_PERF()

    if (cxt_.filter_fcam_) {
      process_pose(msg, parser_.t_fcam_base, "fcam_measurement");
    }

    STOP_PERF("fcam_callback")
  }

  void FilterNode::process_pose(const orca_msgs::msg::FiducialPoseStamped::SharedPtr &msg,
                                const tf2::Transform &t_sensor_base, const std::string &frame_id)
  {
    last_fp_stamp_ = msg->header.stamp;

    // Save observations
    observations_ = mw::Observations{msg->fp.observations};

    // Make sure we have the right filter & state
    create_filter();

    if (!good_pose_) {
      return;
    }

    // If we're receiving poses but not publishing odometry then reset the filter
    if (orca::valid_stamp(last_fp_inlier_stamp_) && last_fp_stamp_ - last_fp_inlier_stamp_ > outlier_timeout_) {
      RCLCPP_DEBUG(get_logger(), "reset filter");
      filter_->reset(msg->fp.pose.pose);
      last_fp_inlier_stamp_ = last_fp_stamp_;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->fp.pose;
    if (filter_->process_message(pose_stamped, observations_)) {
      last_fp_inlier_stamp_ = last_fp_stamp_;
    }
  }

} // namespace orca_filter

//=============================================================================
// Main
//=============================================================================

#ifdef RUN_PERF
void print_mean(std::string msg, int samples[])
{
  int sum = 0;
  for (int i = 0; i < NUM_SAMPLES; ++i) {
    sum += samples[i];
  }
  std::cout << msg << " ave " << sum / NUM_SAMPLES << " microseconds" << std::endl;
}
#endif

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_filter::FilterNode>();

  // Set logger level
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
