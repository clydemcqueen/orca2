#ifndef ORCA_BASE_AUV_NODE_HPP
#define ORCA_BASE_AUV_NODE_HPP

#include "image_geometry/pinhole_camera_model.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "orca_description/parser.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/depth.hpp"
#include "orca_msgs/msg/driver.hpp"
#include "orca_shared/monotonic.hpp"

#include "orca_base/auv_context.hpp"
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

    // Timer period, set by parameter
    std::chrono::milliseconds spin_period_{0};

    // Timeouts, set by parameters
    rclcpp::Duration depth_timeout_{0};
    rclcpp::Duration driver_timeout_{0};
    rclcpp::Duration fp_timeout_{0};

    // Parsed URDF
    orca_description::Parser parser_;

    // Camera model
    image_geometry::PinholeCameraModel fcam_model_;

    // Most recent depth message
    double base_link_z_{};

    // Most recent observations and pose estimate
    mw::FiducialPoseStamped estimate_;

    // AUV operation
    int global_plan_idx_{-1};                     // Count of global plans, starts at 0
    std::shared_ptr<Mission> mission_;            // The mission we're running
    mw::Map map_;                                 // Map of fiducial markers
    nav_msgs::msg::Path estimated_path_;          // Estimate of the actual path

    // Subscriptions
    rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
    rclcpp::Subscription<orca_msgs::msg::Depth>::SharedPtr depth_sub_;
    rclcpp::Subscription<orca_msgs::msg::Driver>::SharedPtr driver_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr fcam_info_sub_;
    rclcpp::Subscription<orca_msgs::msg::FiducialPoseStamped>::SharedPtr fp_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<fiducial_vlam_msgs::msg::Map>::SharedPtr map_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr spin_timer_;

    void validate_parameters();

    bool depth_ok(const rclcpp::Time &t);

    bool driver_ok(const rclcpp::Time &t);

    bool fp_ok(const rclcpp::Time &t);

    bool cam_info_ok();

    bool ready_to_start_mission(const rclcpp::Time &t);

    void timer_callback(bool first);

    /**
     * Abort the active mission and re-publish the message on the output topic
     *
     * If rov_node is active, arrange the nodes and topics serially, e.g.,
     * rov_node -> 'rov_control' topic -> auv_node -> 'control' topic -> driver_node
     *
     * This will prevent rov_node and auv_node from sending conflicting messages to orca_driver
     *
     * @param msg Control message
     */
    void control_callback(orca_msgs::msg::Control::SharedPtr msg);

    void depth_callback(orca_msgs::msg::Depth::SharedPtr msg, bool first);

    void driver_callback(orca_msgs::msg::Driver::SharedPtr msg);

    void fcam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void fp_callback(orca_msgs::msg::FiducialPoseStamped::SharedPtr msg, bool first);

    void map_callback(fiducial_vlam_msgs::msg::Map::SharedPtr msg);

    // Callback wrappers
    monotonic::Timer<AUVNode *>
      timer_cb_{this, &AUVNode::timer_callback};
    monotonic::Monotonic<AUVNode *, orca_msgs::msg::Depth::SharedPtr>
      depth_cb_{this, &AUVNode::depth_callback};
    monotonic::Valid<AUVNode *, orca_msgs::msg::Driver::SharedPtr>
      driver_cb_{this, &AUVNode::driver_callback};
    monotonic::Valid<AUVNode *, sensor_msgs::msg::CameraInfo::SharedPtr>
      fcam_info_cb_{this, &AUVNode::fcam_info_callback};
    monotonic::Monotonic<AUVNode *, orca_msgs::msg::FiducialPoseStamped::SharedPtr>
      fp_cb_{this, &AUVNode::fp_callback};
    monotonic::Valid<AUVNode *, fiducial_vlam_msgs::msg::Map::SharedPtr>
      map_cb_{this, &AUVNode::map_callback};

    // Publications
    rclcpp::Publisher<orca_msgs::msg::Control>::SharedPtr control_pub_;
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

    void auv_advance(const rclcpp::Time &t, const rclcpp::Duration &d);

    void publish_control(const rclcpp::Time &msg_time, const mw::Efforts &efforts);

  public:

    explicit AUVNode();

    ~AUVNode() override;
  };

} // namespace orca_base

#endif // ORCA_BASE_AUV_NODE_HPP
