#ifndef ORCA_BASE_ROV_NODE_HPP
#define ORCA_BASE_ROV_NODE_HPP

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "orca_msgs/action/mission.hpp"
#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/battery.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/leak.hpp"
#include "orca_shared/geometry.hpp"
#include "orca_shared/monotonic.hpp"

#include "orca_base/rov_context.hpp"
#include "orca_base/joystick.hpp"
#include "orca_base/pid.hpp"

namespace orca_base
{

  //=============================================================================
  // ROVNode
  //=============================================================================

  class ROVNode : public rclcpp::Node
  {
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
    ROVContext cxt_;

    // Timeouts will be set by parameters
    rclcpp::Duration baro_timeout_{0};
    rclcpp::Duration joy_timeout_{0};
    std::chrono::milliseconds spin_period_{0};

    // Mode
    uint8_t mode_{orca_msgs::msg::Control::DISARMED};

    // Barometer state
    double pressure_{};

    // Joystick state
    sensor_msgs::msg::Joy joy_msg_;               // Most recent message

    // ROV operation
    std::shared_ptr<pid::Controller> pressure_hold_pid_;

    // Outputs
    int tilt_{};                                  // Camera tilt
    int brightness_{};                            // Lights

    // Subscriptions
    rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
    rclcpp::Subscription<orca_msgs::msg::Battery>::SharedPtr battery_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<orca_msgs::msg::Leak>::SharedPtr leak_sub_;

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

    // Timer callback
    void spin_once();

    // Subscription callbacks
    void baro_callback(orca_msgs::msg::Barometer::SharedPtr msg);

    void battery_callback(orca_msgs::msg::Battery::SharedPtr msg);

    void goal_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg, bool first);

    void leak_callback(orca_msgs::msg::Leak::SharedPtr msg);

    // Callback wrappers
    monotonic::Valid<ROVNode *, orca_msgs::msg::Barometer::SharedPtr> baro_cb_{this, &ROVNode::baro_callback};
    monotonic::Monotonic<ROVNode *, sensor_msgs::msg::Joy::SharedPtr> joy_cb_{this, &ROVNode::joy_callback};

    // Publications
    rclcpp::Publisher<orca_msgs::msg::Control>::SharedPtr control_pub_;

    using MissionAction = orca_msgs::action::Mission;
    using MissionHandle = rclcpp_action::ClientGoalHandle<MissionAction>;

    // Action client
    rclcpp_action::Client<orca_msgs::action::Mission>::SharedPtr mission_client_;

    // Action client callbacks
    void goal_response_callback(std::shared_future<MissionHandle::SharedPtr> future);

    void feedback_callback(MissionHandle::SharedPtr, std::shared_ptr<const MissionAction::Feedback> feedback);

    void result_callback(const MissionHandle::WrappedResult &result);

    void rov_advance(const rclcpp::Time &stamp);

    void publish_control(const rclcpp::Time &msg_time, const orca::Efforts &efforts);

    void disarm(const rclcpp::Time &msg_time);

    void start_rov(const rclcpp::Time &msg_time);

    void start_hold_pressure(const rclcpp::Time &msg_time);

    enum class Mission
    {
      KEEP_STATION, GO_TO_POSE, RANDOM_MARKERS
    };

    void start_mission(const rclcpp::Time &msg_time, Mission mission, const geometry_msgs::msg::Pose &pose = geometry_msgs::msg::Pose{});

    void stop_mission();

  public:

    explicit ROVNode();

    ~ROVNode() override = default;
  };

} // namespace orca_base

#endif //ORCA_BASE_ROV_NODE_HPP
