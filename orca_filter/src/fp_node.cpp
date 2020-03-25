#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "ros2_shared/context_macros.hpp"

#include "orca_description/parser.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_msgs/msg/fiducial_pose_stamped.hpp"
#include "orca_shared/fp.hpp"
#include "orca_shared/monotonic.hpp"

namespace orca_filter
{

  //=============================================================================
  // Parameter(s)
  //=============================================================================

#define FP_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(map_frame, std::string, "map")             /* Map frame  */ \
  CXT_MACRO_MEMBER(base_frame, std::string, "base_link")      /* Base frame  */ \
  \
  CXT_MACRO_MEMBER(fuse_depth, bool, false)                   /* Fuse depth and fiducial messages  */ \
  CXT_MACRO_MEMBER(publish_tf, bool, false)                   /* Publish t_map_base  */ \
  \
  CXT_MACRO_MEMBER(marker_length, double, 0.1778)             /* Marker length in meters  */ \
  CXT_MACRO_MEMBER(fcam_hfov, double, 1.4)                    /* Forward camera horiz field of view in radians  */ \
  CXT_MACRO_MEMBER(fcam_hres, double, 800)                    /* Forward camera horiz resolution in pixels  */ \
/* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

  struct FPContext
  {
    FP_NODE_ALL_PARAMS
  };

  //=============================================================================
  // FPNode subscribes to PoseWithCovarianceStamped and fiducial_vlam::msg::Observations
  // and publishes orca_msgs::msg::FiducialPose
  //=============================================================================

  constexpr int QUEUE_SIZE = 10;

  class FPNode : public rclcpp::Node
  {
    FPContext cxt_;                  // Parameter(s)

    // Parsed URDF
    orca_description::Parser parser_;

    // Most recent depth message
    double base_link_z_{};

    // Message filter subscriptions
    message_filters::Subscriber<fiducial_vlam_msgs::msg::Observations> obs_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> fcam_pose_sub_;

    // Sync pose + observations
    using FiducialPolicy = message_filters::sync_policies::ExactTime<
      fiducial_vlam_msgs::msg::Observations,
      geometry_msgs::msg::PoseWithCovarianceStamped>;
    using FiducialSync = message_filters::Synchronizer<FiducialPolicy>;
    std::shared_ptr<FiducialSync> sync_;

    // Subscriptions
    rclcpp::Subscription<orca_msgs::msg::Depth>::SharedPtr depth_sub_;

    // Publications
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    rclcpp::Publisher<orca_msgs::msg::FiducialPoseStamped>::SharedPtr fp_pub_;

    /**
     * Save the latest depth reading
     * @param msg Depth message
     */
    void depth_callback(orca_msgs::msg::Depth::SharedPtr msg)
    {
      base_link_z_ = msg->z;
    }

    /**
     * Get a synchronized set of messages from vloc: pose and marker observations
     *
     * Guard against invalid or duplicate timestamps
     * vloc poses are sensor_f_map -- transform to base_f_map
     *
     * @param obs_msg Marker observations
     * @param fcam_msg Resulting pose from SolvePnP
     */
    void fiducial_callback(
      const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &obs_msg,
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &fcam_msg)
    {
      // Keep track of the previous time
      static rclcpp::Time prev_time{0, 0, RCL_ROS_TIME};

      // The current time
      rclcpp::Time curr_time = obs_msg->header.stamp;

      // Skip first message
      if (!monotonic::valid(prev_time)) {
        prev_time = curr_time;
        return;
      }

      // Ignore invalid timestamps
      if (!monotonic::valid(curr_time)) {
        return;
      }

      // Ignore duplicates, or time going backwards
      // (Possible in simulation if we're running ROS time, see notes in orca_gazebo README)
      if (curr_time <= prev_time) {
        return;
      }

      // Convert pose to transform
      tf2::Transform t_map_sensor;
      tf2::fromMsg(fcam_msg->pose.pose, t_map_sensor);

      // Multiply transforms to get t_map_base
      tf2::Transform t_map_base = t_map_sensor * parser_.t_fcam_base;

      // Convert transform back to pose
      geometry_msgs::msg::PoseWithCovariance base_f_map;
      toMsg(t_map_base, base_f_map.pose);
      base_f_map.covariance = fcam_msg->pose.covariance;  // TODO rotate covariance

      // Publish tf
      if (cxt_.publish_tf_ && tf_pub_->get_subscription_count() > 0) {
        // Build transform message
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = curr_time;
        transform.header.frame_id = cxt_.map_frame_;
        transform.child_frame_id = cxt_.base_frame_;
        transform.transform = toMsg(t_map_base);

        // TF messages can have multiple transforms, we have just 1
        tf2_msgs::msg::TFMessage tf_message;
        tf_message.transforms.emplace_back(transform);

        tf_pub_->publish(tf_message);
      }

      // Resulting fiducial pose
      orca_msgs::msg::FiducialPoseStamped fp_msg{};
      fp_msg.header = fcam_msg->header;
      fp_msg.fp.pose = base_f_map;

      // "Fuse" depth and fiducial messages
      if (cxt_.fuse_depth_) {
        fp_msg.fp.pose.pose.position.z = base_link_z_;
      }

      // Convert observations
      orca::vlam_to_orca(obs_msg, fp_msg.fp.observations, cxt_.marker_length_, cxt_.fcam_hfov_, cxt_.fcam_hres_);

      // Publish
      fp_pub_->publish(fp_msg);

      // Update prev_time
      prev_time = curr_time;
    }

    void fiducial_drop_callback(
      const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &obs_msg,
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &fcam_msg)
    {
      if (obs_msg) {
        RCLCPP_WARN(get_logger(), "drop: have an observation but no odometry");
      } else {
        RCLCPP_WARN(get_logger(), "drop: have odometry but no observation");
      }
    }

    // Validate parameters
    void validate_parameters()
    {
      if (cxt_.fuse_depth_) {
        using namespace std::placeholders;
        auto depth_cb = std::bind(&FPNode::depth_callback, this, _1);
        depth_sub_ = create_subscription<orca_msgs::msg::Depth>("depth", QUEUE_SIZE, depth_cb);
      } else {
        depth_sub_.reset();
      }

      if (cxt_.publish_tf_) {
        tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", QUEUE_SIZE);
      } else {
        tf_pub_.reset();
      }
    }

  public:

    explicit FPNode() : Node{"fp_node"}
    {
      // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
      CXT_MACRO_INIT_PARAMETERS(FP_NODE_ALL_PARAMS, validate_parameters)

      // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
      CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), FP_NODE_ALL_PARAMS, validate_parameters)

      // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
      FP_NODE_ALL_PARAMS

      // Message filter subscriptions
      obs_sub_.subscribe(this, "fiducial_observations"); // Default qos is reliable, queue=10
      fcam_pose_sub_.subscribe(this, "fcam_f_map"); // Default qos is reliable, queue=10

      // Start sync
      using namespace std::placeholders;
      sync_.reset(new FiducialSync(FiducialPolicy(QUEUE_SIZE), obs_sub_, fcam_pose_sub_));
      sync_->registerCallback(std::bind(&FPNode::fiducial_callback, this, _1, _2));
      sync_->registerDropCallback(std::bind(&FPNode::fiducial_drop_callback, this, _1, _2));

      // Publication
      fp_pub_ = create_publisher<orca_msgs::msg::FiducialPoseStamped>("fp", QUEUE_SIZE);

      // Parse URDF
      if (!parser_.parse()) {
        RCLCPP_ERROR(get_logger(), "can't parse URDF %s", orca_description::filename);
      }

      RCLCPP_INFO(get_logger(), "fp_node ready");
    }

    ~FPNode() override = default;
  };

} // namespace orca_filter

//=============================================================================
// Main
//=============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_filter::FPNode>();

  // Set logger level
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}