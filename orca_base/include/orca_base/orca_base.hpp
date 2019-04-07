#ifndef ORCA_BASE_H
#define ORCA_BASE_H

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
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
#include "orca_base/joystick.hpp"
#include "orca_base/orca_model.hpp"
#include "orca_base/pid.hpp"
#include "orca_base/orca_mission.hpp"

namespace orca_base {

constexpr const bool headingHoldMode(uint8_t mode) { return mode == orca_msgs::msg::Control::HOLD_H || mode == orca_msgs::msg::Control::HOLD_HD; }
constexpr const bool depthHoldMode(uint8_t mode) { return mode == orca_msgs::msg::Control::HOLD_D || mode == orca_msgs::msg::Control::HOLD_HD; }
constexpr const bool rovMode(uint8_t mode) { return mode == orca_msgs::msg::Control::MANUAL || mode == orca_msgs::msg::Control::HOLD_H || mode == orca_msgs::msg::Control::HOLD_D || mode == orca_msgs::msg::Control::HOLD_HD; }
constexpr const bool auvMode(uint8_t mode) { return mode == orca_msgs::msg::Control::MISSION; }

// OrcaBase provides basic ROV and AUV functions, including joystick operation and waypoint navigation.
class OrcaBase: public rclcpp::Node
{
private:
  // Joystick assignments
  int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
  int joy_axis_forward_ = JOY_AXIS_LEFT_FB;
  int joy_axis_strafe_ = JOY_AXIS_RIGHT_LR;
  int joy_axis_vertical_ = JOY_AXIS_RIGHT_FB;
  int joy_axis_yaw_trim_ = JOY_AXIS_TRIM_LR;
  int joy_axis_depth_trim_ = JOY_AXIS_TRIM_FB;

  int joy_button_disarm_ = JOY_BUTTON_VIEW;
  int joy_button_arm_ = JOY_BUTTON_MENU;
  int joy_button_manual_ = JOY_BUTTON_A;
  int joy_button_hold_h_ = JOY_BUTTON_X;
  int joy_button_hold_d_ = JOY_BUTTON_B;
  int joy_button_hold_hd_ = JOY_BUTTON_Y;
  int joy_button_tilt_down_ = JOY_BUTTON_LEFT_BUMPER;
  int joy_button_tilt_up_ = JOY_BUTTON_RIGHT_BUMPER;
  int joy_button_bright_ = JOY_BUTTON_LEFT_STICK;
  int joy_button_dim_ = JOY_BUTTON_RIGHT_STICK;

  double inc_yaw_;
  double inc_depth_;
  int inc_tilt_;
  int inc_lights_;
  float input_dead_band_;
  double yaw_pid_dead_band_;
  double depth_pid_dead_band_;
  tf2::Matrix3x3 imu_rotation_;

  // General state
  bool simulation_;                                 // True if we're in a simulation
  rclcpp::Time prev_loop_time_;                     // Last time spinOnce was called
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

  // Depth controller
  pid::Controller depth_controller_;
  double depth_adjustment_;
  double depth_state_;
  double depth_setpoint_;  // TODO get from controller

  // Joystick gain (attenuation), range 0.0 (ignore joystick) to 1.0 (no attenuation)
  double xy_gain_;
  double yaw_gain_;
  double vertical_gain_;

  // Thruster effort from joystick or pid controllers (yaw and depth), ranges from 1.0 for forward to -1.0 for reverse
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
  void baroCallback(const orca_msgs::msg::Barometer::SharedPtr msg);
  void batteryCallback(const orca_msgs::msg::Battery::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void gpsCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void leakCallback(const orca_msgs::msg::Leak::SharedPtr msg);
  void odomLocalCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Publications
  rclcpp::Publisher<orca_msgs::msg::Control>::SharedPtr control_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr thrust_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_estimated_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_ground_truth_pub_;

  // Helpers
  void publishControl();
  void publishOdom();
  void setMode(uint8_t new_mode);
  bool holdingHeading() { return headingHoldMode(mode_); };
  bool holdingDepth() { return depthHoldMode(mode_); };
  bool rovOperation() { return rovMode(mode_); };
  bool auvOperation() { return auvMode(mode_); };

public:
  explicit OrcaBase();
  ~OrcaBase() {}; // Suppress default copy and move constructors

  void spinOnce();
};

} // namespace orca_base

#endif // ORCA_BASE_H
