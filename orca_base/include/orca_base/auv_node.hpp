#ifndef ORCA_BASE_AUV_NODE_HPP
#define ORCA_BASE_AUV_NODE_HPP

#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "orca_description/parser.hpp"
#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/battery.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/leak.hpp"
#include "orca_shared/baro.hpp"
#include "orca_shared/monotonic.hpp"

#include "orca_base/auv_context.hpp"
#include "orca_base/map.hpp"
#include "orca_base/mission.hpp"

namespace orca_base
{

  //=============================================================================
  // AUVNode
  //=============================================================================

  class AUVNode : public rclcpp::Node
  {
    // Parameters and dynamics model
    AUVContext cxt_;

    // Timeouts will be set by parameters
    rclcpp::Duration baro_timeout_{0};
    rclcpp::Duration fp_timeout_{0};
    std::chrono::milliseconds spin_period_{0};

    // Parsed URDF
    orca_description::Parser parser_;

    // Barometer state
    orca::Barometer barometer_{};
    double base_link_z_{};

    // Camera model
    image_geometry::PinholeCameraModel fcam_model_;

    // Observations and pose estimate
    FPStamped estimate_;

    // AUV operation
    int global_plan_idx_{-1};                     // Count of global plans, starts at 0
    std::shared_ptr<Mission> mission_;            // The mission we're running
    Map map_;                                     // Map of fiducial markers
    nav_msgs::msg::Path estimated_path_;          // Estimate of the actual path

    // Subscriptions
    rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
    rclcpp::Subscription<orca_msgs::msg::Battery>::SharedPtr battery_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr fcam_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr fcam_info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<orca_msgs::msg::Leak>::SharedPtr leak_sub_;
    rclcpp::Subscription<fiducial_vlam_msgs::msg::Map>::SharedPtr map_sub_;

    // Sync pose + observations
    // These will only be sent if markers were found
    message_filters::Subscriber<fiducial_vlam_msgs::msg::Observations> obs_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> fcam_pose_sub_;
    using FiducialPolicy = message_filters::sync_policies::ExactTime<
      fiducial_vlam_msgs::msg::Observations,
      geometry_msgs::msg::PoseWithCovarianceStamped>;
    using FiducialSync = message_filters::Synchronizer<FiducialPolicy>;
    std::shared_ptr<FiducialSync> fiducial_sync_;

    // Timer
    rclcpp::TimerBase::SharedPtr spin_timer_;

    // Validate parameters
    void validate_parameters();

    bool baro_ok(const rclcpp::Time &t);

    bool fp_ok(const rclcpp::Time &t);

    bool cam_info_ok();

    bool ready_to_start_mission();

    // Timer callback
    void spin_once();

    // Subscription callbacks
    void baro_callback(orca_msgs::msg::Barometer::SharedPtr msg);

    void battery_callback(orca_msgs::msg::Battery::SharedPtr msg);

    void fcam_image_callback(sensor_msgs::msg::Image::SharedPtr msg);

    void fcam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void leak_callback(orca_msgs::msg::Leak::SharedPtr msg);

    void map_callback(fiducial_vlam_msgs::msg::Map::SharedPtr msg);

    void fiducial_callback(
      const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &obs_msg,
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &fcam_msg);

    // Callback wrappers
    monotonic::Valid<AUVNode *, orca_msgs::msg::Barometer::SharedPtr> baro_cb_{this, &AUVNode::baro_callback};
    monotonic::Valid<AUVNode *, sensor_msgs::msg::Image::SharedPtr> fcam_image_cb_{this,
                                                                                   &AUVNode::fcam_image_callback};
    monotonic::Valid<AUVNode *, sensor_msgs::msg::CameraInfo::SharedPtr> fcam_info_cb_{this,
                                                                                       &AUVNode::fcam_info_callback};
    monotonic::Valid<AUVNode *, fiducial_vlam_msgs::msg::Map::SharedPtr> map_cb_{this, &AUVNode::map_callback};

    // Publications
    rclcpp::Publisher<orca_msgs::msg::Control>::SharedPtr control_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fcam_predicted_obs_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr esimated_path_pub_;             // Actual path
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planned_pose_pub_;  // Planned pose
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_path_pub_;               // Planned path
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

    using MissionAction = orca_msgs::action::Mission;
    using MissionHandle = rclcpp_action::ServerGoalHandle<MissionAction>;

    // Mission server
    rclcpp_action::Server<MissionAction>::SharedPtr mission_server_;

    // Mission callbacks
    rclcpp_action::GoalResponse
    mission_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MissionAction::Goal> goal);

    rclcpp_action::CancelResponse mission_cancel(std::shared_ptr<MissionHandle> goal_handle);

    void mission_accepted(std::shared_ptr<MissionHandle> goal_handle);

    void abort_mission(const rclcpp::Time &msg_time);

    void auv_advance(const rclcpp::Time &msg_time, const rclcpp::Duration &d);

    void publish_control(const rclcpp::Time &msg_time, const orca::Efforts &efforts);

    void write_status(cv::Mat &image);

  public:

    explicit AUVNode();

    ~AUVNode() override = default;
  };

} // namespace orca_base

#endif // ORCA_BASE_AUV_NODE_HPP
