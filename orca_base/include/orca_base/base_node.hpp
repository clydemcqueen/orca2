#ifndef ORCA_BASE_BASE_NODE_HPP
#define ORCA_BASE_BASE_NODE_HPP

#include "builtin_interfaces/msg/time.hpp"
#include "fiducial_vlam_msgs/msg/map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/battery.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/leak.hpp"

#include "orca_base/controller.hpp"
#include "orca_base/joystick.hpp"

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
  const int joy_button_mission_ = JOY_BUTTON_LOGO;
  const int joy_button_tilt_down_ = JOY_BUTTON_LEFT_BUMPER;
  const int joy_button_tilt_up_ = JOY_BUTTON_RIGHT_BUMPER;
  const int joy_button_bright_ = JOY_BUTTON_LEFT_STICK;
  const int joy_button_dim_ = JOY_BUTTON_RIGHT_STICK;

  // Parameters
  BaseContext cxt_;

  // General state
  uint8_t mode_;                              // Operating mode

  // Barometer state
  rclcpp::Time baro_time_;                    // Time of most recent message
  double z_initial_;                          // First z value, used to adjust barometer
  double z_;                                  // Z from barometer

  // IMU state
  tf2::Matrix3x3 t_imu_base_;                 // Static transform from the base frame to the imu frame
  rclcpp::Time imu_time_;                     // Time of most recent message
  double yaw_;                                // Yaw from IMU
  double stability_;                          // Roll and pitch stability, 1.0 (flat) to 0.0 (>90 degree tilt)

  // Joystick state
  sensor_msgs::msg::Joy joy_msg_;             // Most recent message

  // Odometry state
  PoseStamped filtered_pose_;                 // Estimated pose

  // ROV operation
  std::shared_ptr<pid::Controller> rov_yaw_pid_;
  std::shared_ptr<pid::Controller> rov_z_pid_;

  // AUV operation
  std::shared_ptr<Controller> controller_;    // The mission we're running
  fiducial_vlam_msgs::msg::Map map_;          // Map of fiducial markers
  nav_msgs::msg::Path filtered_path_;         // Estimate of the actual path (from filtered_pose_)

  // Outputs
  Efforts efforts_;                           // Thruster forces
  int tilt_;                                  // Camera tilt
  int brightness_;                            // Lights

  // Subscriptions
  rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Subscription<orca_msgs::msg::Battery >::SharedPtr battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<orca_msgs::msg::Leak>::SharedPtr leak_sub_;
  rclcpp::Subscription<fiducial_vlam_msgs::msg::Map>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Callbacks
  void baro_callback(const orca_msgs::msg::Barometer::SharedPtr msg);
  void battery_callback(const orca_msgs::msg::Battery::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void leak_callback(const orca_msgs::msg::Leak::SharedPtr msg);
  void map_callback(const fiducial_vlam_msgs::msg::Map::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Publications
  rclcpp::Publisher<orca_msgs::msg::Control>::SharedPtr control_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr planned_odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr thrust_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr filtered_path_pub_;

  // Helpers
  void publish_control(const rclcpp::Time &msg_time);
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
