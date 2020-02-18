#ifndef ORCA_BASE_BASE_NODE_HPP
#define ORCA_BASE_BASE_NODE_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "fiducial_vlam_msgs/msg/observations.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "orca_description/parser.hpp"
#include "orca_msgs/action/mission.hpp"
#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/battery.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/leak.hpp"
#include "orca_shared/baro.hpp"
#include "orca_shared/monotonic.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/joystick.hpp"
#include "orca_base/map.hpp"
#include "orca_base/mission.hpp"

//using namespace std::chrono_literals;

namespace orca_base
{

  //=============================================================================
  // Thrusters
  //=============================================================================

  struct Thruster
  {
    std::string frame_id;   // URDF link frame id
    bool ccw;               // True if counterclockwise
    double forward_factor;
    double strafe_factor;
    double yaw_factor;
    double vertical_factor;
  };

  //=============================================================================
  // BaseNode
  //=============================================================================

  class BaseNode : public rclcpp::Node
  {
  private:

    // Thrusters, order must match the order of the <thruster> tags in the URDF
    const std::vector<Thruster> THRUSTERS = {
      {"t200_link_front_right",    false, 1.0, 1.0,  1.0,  0.0},
      {"t200_link_front_left",     false, 1.0, -1.0, -1.0, 0.0},
      {"t200_link_rear_right",     true,  1.0, -1.0, 1.0,  0.0},
      {"t200_link_rear_left",      true,  1.0, 1.0,  -1.0, 0.0},
      {"t200_link_vertical_right", false, 0.0, 0.0,  0.0,  1.0},
      {"t200_link_vertical_left",  true,  0.0, 0.0,  0.0,  -1.0},
    };

    // Joystick assignments
    const int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
    const int joy_axis_forward_ = JOY_AXIS_LEFT_FB;
    const int joy_axis_strafe_ = JOY_AXIS_RIGHT_LR;
    const int joy_axis_vertical_ = JOY_AXIS_RIGHT_FB;
    const int joy_axis_z_trim_ = JOY_AXIS_TRIM_FB;

    const int joy_button_disarm_ = JOY_BUTTON_VIEW;
    const int joy_button_arm_ = JOY_BUTTON_MENU;
    const int joy_button_rov_ = JOY_BUTTON_A;
    const int joy_button_rov_hold_pressure_ = JOY_BUTTON_B;
    const int joy_button_auv_keep_station_ = JOY_BUTTON_X;
    const int joy_button_auv_random_ = JOY_BUTTON_Y;

    const int joy_button_tilt_down_ = JOY_BUTTON_LEFT_BUMPER;
    const int joy_button_tilt_up_ = JOY_BUTTON_RIGHT_BUMPER;
    const int joy_button_bright_ = JOY_BUTTON_LEFT_STICK;
    const int joy_button_dim_ = JOY_BUTTON_RIGHT_STICK;

    // Parameters and dynamics model
    BaseContext cxt_;

    // Timeouts will be set by parameters
    rclcpp::Duration baro_timeout_{0};
    rclcpp::Duration fp_timeout_{0};
    rclcpp::Duration joy_timeout_{0};
    std::chrono::milliseconds spin_period_{0};

    // Parsed URDF
    orca_description::Parser parser_;

    // Mode
    uint8_t mode_{orca_msgs::msg::Control::DISARMED};

    // Barometer state
    double pressure_{};
    orca::Barometer barometer_{0}; // Init at surface
    double z_{};

    // Joystick state
    sensor_msgs::msg::Joy joy_msg_;               // Most recent message

    // Odometry state
    double stability_{1.0};                       // Roll and pitch stability, 1.0 (flat) to 0.0 (>90 degree tilt)

    // Camera model
    image_geometry::PinholeCameraModel fcam_model_;

    // Plan
    orca::FPStamped plan_;

    // Observations and pose estimate
    orca::FPStamped estimate_;

    // ROV operation
    std::shared_ptr<pid::Controller> pressure_hold_pid_;

    // AUV operation
    std::shared_ptr<Mission> mission_;            // The mission we're running
    Map map_;                                     // Map of fiducial markers
    nav_msgs::msg::Path estimated_path_;          // Estimate of the actual path

    // Outputs
    int tilt_{};                                  // Camera tilt
    int brightness_{};                            // Lights

    // Subscriptions
    rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
    rclcpp::Subscription<orca_msgs::msg::Battery>::SharedPtr battery_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr fcam_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr fcam_info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
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

    // State testers
    bool holding_pressure();

    bool rov_mode();

    bool auv_mode();

    bool baro_ok(const rclcpp::Time &t);

    bool joy_ok(const rclcpp::Time &t);

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

    void goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg, bool first);

    void leak_callback(orca_msgs::msg::Leak::SharedPtr msg);

    void map_callback(fiducial_vlam_msgs::msg::Map::SharedPtr msg);

    void fiducial_callback(
      const fiducial_vlam_msgs::msg::Observations::ConstSharedPtr &obs_msg,
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &fcam_msg);

    // Callback wrappers
    monotonic::Valid<BaseNode *, orca_msgs::msg::Barometer::SharedPtr> baro_cb_{this, &BaseNode::baro_callback};
    monotonic::Valid<BaseNode *, sensor_msgs::msg::Image::SharedPtr> fcam_image_cb_{this,
                                                                                    &BaseNode::fcam_image_callback};
    monotonic::Valid<BaseNode *, sensor_msgs::msg::CameraInfo::SharedPtr> fcam_info_cb_{this,
                                                                                        &BaseNode::fcam_info_callback};
    monotonic::Monotonic<BaseNode *, sensor_msgs::msg::Joy::SharedPtr> joy_cb_{this, &BaseNode::joy_callback};
    monotonic::Valid<BaseNode *, fiducial_vlam_msgs::msg::Map::SharedPtr> map_cb_{this, &BaseNode::map_callback};

    // Publications
    rclcpp::Publisher<orca_msgs::msg::Control>::SharedPtr control_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fcam_predicted_obs_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr esimated_path_pub_;             // Actual path
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planned_pose_pub_;  // Planned pose
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_path_pub_;               // Planned path
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr thrust_marker_pub_;

    // Mission server
    rclcpp_action::Server<orca_msgs::action::Mission>::SharedPtr mission_server_;

    // Mission callbacks
    rclcpp_action::GoalResponse
    mission_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const orca_msgs::action::Mission::Goal> goal);

    rclcpp_action::CancelResponse
    mission_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle);

    void mission_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> goal_handle);

    void rov_advance(const rclcpp::Time &stamp);

    void auv_advance(const rclcpp::Time &msg_time, const rclcpp::Duration &d);

    void all_stop(const rclcpp::Time &msg_time);

    void publish_control(const rclcpp::Time &msg_time, const orca::Pose &error, const orca::Efforts &efforts);

    void disarm(const rclcpp::Time &msg_time);

    void set_mode(const rclcpp::Time &msg_time, uint8_t new_mode, const orca::FP &goal = {},
                  const std::vector<geometry_msgs::msg::Pose> &msgs = {},
                  const std::shared_ptr<rclcpp_action::ServerGoalHandle<orca_msgs::action::Mission>> &goal_handle = nullptr);

    void write_status(cv::Mat &image);

  public:
    explicit BaseNode();

    ~BaseNode() override = default;
  };

} // namespace orca_base

#endif // ORCA_BASE_BASE_NODE_HPP
