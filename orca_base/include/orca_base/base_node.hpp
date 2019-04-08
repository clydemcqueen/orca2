#ifndef ORCA_BASE_BASE_NODE_HPP
#define ORCA_BASE_BASE_NODE_HPP

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "orca_base/base_context.hpp"
#include "orca_base/joystick.hpp"
#include "orca_base/model.hpp"
#include "orca_base/pid.hpp"
#include "orca_base/mission.hpp"

#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/battery.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/leak.hpp"

namespace orca_base {

bool is_yaw_hold_mode(uint8_t mode)
{
  using orca_msgs::msg::Control;
  return mode == Control::HOLD_H || mode == Control::HOLD_HD;
}

bool is_z_hold_mode(uint8_t mode)
{
  using orca_msgs::msg::Control;
  return mode == Control::HOLD_D || mode == Control::HOLD_HD;
}

bool is_rov_mode(uint8_t mode)
{
  using orca_msgs::msg::Control;
  return mode == Control::MANUAL || mode == Control::HOLD_H || mode == Control::HOLD_D || mode == Control::HOLD_HD;
}

bool is_auv_mode(uint8_t mode)
{
  using orca_msgs::msg::Control;
  return mode == Control::MISSION;
}

// BaseNode provides basic ROV and AUV functions, including joystick operation and waypoint navigation.
class BaseNode: public rclcpp::Node
{
private:
  // Joystick assignments
  const int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
  const int joy_axis_forward_ = JOY_AXIS_LEFT_FB;
  const int joy_axis_strafe_ = JOY_AXIS_RIGHT_LR;
  const int joy_axis_vertical_ = JOY_AXIS_RIGHT_FB;
  const int joy_axis_yaw_trim_ = JOY_AXIS_TRIM_LR;
  const int joy_axis_z_trim_ = JOY_AXIS_TRIM_FB;

  const int joy_button_disarm_ = JOY_BUTTON_VIEW;
  const int joy_button_arm_ = JOY_BUTTON_MENU;
  const int joy_button_manual_ = JOY_BUTTON_A;
  const int joy_button_hold_h_ = JOY_BUTTON_X;
  const int joy_button_hold_d_ = JOY_BUTTON_B;
  const int joy_button_hold_hd_ = JOY_BUTTON_Y;
  const int joy_button_tilt_down_ = JOY_BUTTON_LEFT_BUMPER;
  const int joy_button_tilt_up_ = JOY_BUTTON_RIGHT_BUMPER;
  const int joy_button_bright_ = JOY_BUTTON_LEFT_STICK;
  const int joy_button_dim_ = JOY_BUTTON_RIGHT_STICK;

  // Parameters
  BaseContext cxt_;

  // How IMU is positioned on the AUV TODO use t_x_y notation
  tf2::Matrix3x3 imu_rotation_;

  // General state
  bool simulation_;                                 // True if we're in a simulation
  rclcpp::Time prev_loop_time_;                     // Last time spin_once was called
  rclcpp::Time prev_joy_time_;                      // Last time we heard from the joystick
  uint8_t mode_;                                    // Operating mode

  // State estimation and planning
  OrcaOdometry odom_plan_;                          // Planned state
  OrcaPose odom_local_;                             // Estimated state
  OrcaPose odom_ground_truth_;                      // Ground truth

  // Current mission
  std::unique_ptr<BaseMission> mission_;            // The mission we're running
  nav_msgs::msg::Path mission_plan_path_;           // The planned path (data from odom_plan_)
  nav_msgs::msg::Path mission_estimated_path_;      // Best estimate of the actual path (data from odom_local_)
  nav_msgs::msg::Path mission_ground_truth_path_;   // Ground truth (data from odom_ground_truth_)

  // Sensor status
  bool barometer_ready_;                            // True if we're receiving barometer messages
  bool gps_ready_;                                  // True if we're receiving GPS messages
  bool ground_truth_ready_;                         // True if we're receiving ground truth messages
  bool imu_ready_;                                  // True if we're receiving IMU messages

  // IMU
  tf2::Matrix3x3 base_orientation_;   // Orientation
  double stability_;                  // Roll and pitch stability from 1.0 (flat) to 0.0 (90 tilt or worse)

  // Yaw controller
  pid::Controller yaw_controller_;
  double yaw_state_;
  double yaw_setpoint_;  // TODO get from controller

  // Z controller
  pid::Controller z_controller_;
  double z_adjustment_;
  double z_state_;
  double z_setpoint_;  // TODO get from controller

  // Thruster effort from joystick or pid controllers (yaw and z), ranges from 1.0 for forward to -1.0 for reverse
  OrcaEfforts efforts_;

  // Camera tilt
  int tilt_;

  // Lights
  int brightness_;

  // Subscriptions
  rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Subscription<orca_msgs::msg::Battery >::SharedPtr battery_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<orca_msgs::msg::Leak >::SharedPtr leak_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_local_sub_;

  // Callbacks
  void baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg);
  void battery_callback(const orca_msgs::msg::Battery::SharedPtr msg);
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void gps_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void leak_callback(const orca_msgs::msg::Leak::SharedPtr msg);
  void odom_local_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Publications
  rclcpp::Publisher<orca_msgs::msg::Control>::SharedPtr control_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr thrust_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_estimated_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_ground_truth_pub_;

  // Helpers
  void publish_control();
  void publish_odom();
  void set_mode(uint8_t new_mode);
  bool holding_yaw() { return is_yaw_hold_mode(mode_); };
  bool holding_z() { return is_z_hold_mode(mode_); };
  bool rov_mode() { return is_rov_mode(mode_); };
  bool auv_mode() { return is_auv_mode(mode_); };

public:
  explicit BaseNode();
  ~BaseNode() {}; // Suppress default copy and move constructors

  void spin_once();
};

} // namespace orca_base

#endif // ORCA_BASE_BASE_NODE_HPP
