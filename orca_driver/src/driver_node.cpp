#include "orca_driver/driver_node.hpp"

namespace orca_driver
{

  const rclcpp::Duration CONTROL_TIMEOUT{RCL_S_TO_NS(1)}; // All-stop if control messages stop TODO move to param

  bool valid(const rclcpp::Time &t)
  {
    return t.nanoseconds() > 0;
  }

  //=============================================================================
  // DriverNode
  //=============================================================================

  constexpr int QUEUE_SIZE = 10;

  DriverNode::DriverNode() :
    Node{"driver_node"}
  {
    // Suppress IDE warnings
    (void) control_sub_;
    (void) spin_timer_;

    // Get parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(DRIVER_NODE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), DRIVER_NODE_ALL_PARAMS, validate_parameters)

    // Get thruster parameters
    RCLCPP_INFO(get_logger(), "configuring %d thrusters:", cxt_.num_thrusters_);
    for (int i = 0; i < cxt_.num_thrusters_; ++i) {
      Thruster t;
      get_parameter_or("thruster_" + std::to_string(i + 1) + "_channel", t.channel_, i); // No checks for conflicts!
      get_parameter_or("thruster_" + std::to_string(i + 1) + "_reverse", t.reverse_, false);
      thrusters_.push_back(t);
      RCLCPP_INFO(get_logger(), "thruster %d on channel %d %s", i + 1, t.channel_, t.reverse_ ? "(reversed)" : "");
    }

    // Publish driver status messages
    driver_pub_ = create_publisher<orca_msgs::msg::Driver>("driver_status", QUEUE_SIZE);

    // Subscribe to control messages
    using std::placeholders::_1;
    auto control_cb = std::bind(&DriverNode::control_callback, this, _1);
    control_sub_ = create_subscription<orca_msgs::msg::Control>("control", QUEUE_SIZE, control_cb);

    // Spin timer
    // TODO move to param
    // TODO run at 100ms... or change how auv_advance works
    using namespace std::chrono_literals;
    spin_timer_ = create_wall_timer(500ms, std::bind(&DriverNode::timer_callback, this));
  }

  void DriverNode::validate_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    DRIVER_NODE_ALL_PARAMS
  }

  void DriverNode::set_status(uint8_t status)
  {
    if (status != driver_msg_.status) {
      driver_msg_.status = status;

      led_ready_.setBrightness(0);
      led_mission_.setBrightness(0);
      led_problem_.setBrightness(0);

      switch (driver_msg_.status) {
        case orca_msgs::msg::Driver::STATUS_OK:
          led_ready_.setBrightness(led_ready_.readMaxBrightness() / 2);
          break;
        case orca_msgs::msg::Driver::STATUS_OK_MISSION:
          led_mission_.setBrightness(led_mission_.readMaxBrightness() / 2);
          break;
        case orca_msgs::msg::Driver::STATUS_ABORT:
          led_problem_.setBrightness(led_problem_.readMaxBrightness() / 2);
          break;
        default:
          break;
      }
    }
  }

  void DriverNode::control_callback(const orca_msgs::msg::Control::SharedPtr msg)
  {
    if (!valid(control_msg_time_)) {
      RCLCPP_INFO(get_logger(), "receiving control messages");
    }

    control_msg_time_ = msg->header.stamp;

    if (maestro_.ready()) {
      set_status(msg->mode == msg->AUV ? orca_msgs::msg::Driver::STATUS_OK_MISSION : orca_msgs::msg::Driver::STATUS_OK);

      if (!maestro_.setPWM(static_cast<uint8_t>(cxt_.tilt_channel_), msg->camera_tilt_pwm)) {
        RCLCPP_ERROR(get_logger(), "failed to set camera tilt");
      }
      if (!maestro_.setPWM(static_cast<uint8_t>(cxt_.lights_channel_), msg->brightness_pwm)) {
        RCLCPP_ERROR(get_logger(), "failed to set brightness");
      }

      for (size_t i = 0; i < thrusters_.size(); ++i) {
        uint16_t pwm = msg->thruster_pwm[i];

        // Compensate for ESC programming errors
        if (thrusters_[i].reverse_) {
          pwm = static_cast<uint16_t>(3000 - pwm);
        }

        if (!maestro_.setPWM(static_cast<uint8_t>(thrusters_[i].channel_), pwm)) {
          RCLCPP_ERROR(get_logger(), "failed to set thruster %d", i);
        }
      }
    }
  }

  void DriverNode::timer_callback()
  {
    if (!maestro_.ready()) {
      return;
    }

    if (!read_battery() || !read_leak() || driver_msg_.low_battery || driver_msg_.leak_detected) {
      // Huge problem, we're done
      abort();
      return;
    }

    if (valid(control_msg_time_) && now() - control_msg_time_ > CONTROL_TIMEOUT) {
      // We were receiving control messages, but they stopped.
      // This is normal, but it might also indicate that a node died.
      RCLCPP_INFO(get_logger(), "control timeout");
      control_msg_time_ = rclcpp::Time();
      all_stop();
    }

    driver_msg_.header.stamp = now();
    driver_pub_->publish(driver_msg_);
  }

  // Read battery sensor, return true if successful
  bool DriverNode::read_battery()
  {
    double value = 0.0;
    if (maestro_.ready() && maestro_.getAnalog(static_cast<uint8_t>(cxt_.voltage_channel_), value)) {
      driver_msg_.voltage = value * cxt_.voltage_multiplier_;
      driver_msg_.low_battery = static_cast<uint8_t>(driver_msg_.voltage < cxt_.voltage_min_);
      if (driver_msg_.low_battery) {
        RCLCPP_ERROR(get_logger(), "battery voltage %g is below minimum %g", driver_msg_.voltage, cxt_.voltage_min_);
      }
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "can't read battery");
      driver_msg_.voltage = 0;
      driver_msg_.low_battery = 1;
      return false;
    }
  }

  // Read leak sensor, return true if successful
  bool DriverNode::read_leak()
  {
    bool value = 0.0;
    if (maestro_.ready() && maestro_.getDigital(static_cast<uint8_t>(cxt_.leak_channel_), value)) {
      driver_msg_.leak_detected = static_cast<uint8_t>(value);
      if (driver_msg_.leak_detected) {
        RCLCPP_ERROR(get_logger(), "leak detected");
      }
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "can't read leak sensor");
      driver_msg_.leak_detected = 1;
      return false;
    }
  }

  // Stop all motion
  void DriverNode::all_stop()
  {
    RCLCPP_INFO(get_logger(), "all stop");
    if (maestro_.ready()) {
      for (size_t i = 0; i < thrusters_.size(); ++i) {
        maestro_.setPWM(static_cast<uint8_t>(thrusters_[i].channel_), orca_msgs::msg::Control::THRUST_STOP);
      }
    }
  }

  // Abnormal exit
  void DriverNode::abort()
  {
    RCLCPP_ERROR(get_logger(), "aborting dive");
    set_status(orca_msgs::msg::Driver::STATUS_ABORT);
    all_stop();
    maestro_.disconnect();
  }

  // Connect to Maestro and run pre-dive checks, return true if successful
  bool DriverNode::connect_controller()
  {
    maestro_.connect(cxt_.maestro_port_);
    if (!maestro_.ready()) {
      RCLCPP_ERROR(get_logger(), "can't open port %s, connected? member of dialout?", cxt_.maestro_port_.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "port %s open", cxt_.maestro_port_.c_str());

    // When the Maestro boots, it should set all thruster channels to 1500.
    // But on a system restart it might be a bad state. Force an all-stop.
    all_stop();

    // Check to see that all thrusters are stopped.
    for (size_t i = 0; i < thrusters_.size(); ++i) {
      uint16_t value = 0;
      maestro_.getPWM(static_cast<uint8_t>(thrusters_[i].channel_), value);
      RCLCPP_INFO(get_logger(), "thruster %d is set at %d", i + 1, value);
      if (value != orca_msgs::msg::Control::THRUST_STOP) {
        RCLCPP_ERROR(get_logger(), "thruster %d didn't initialize properly (and possibly others)", i + 1);
        maestro_.disconnect();
        return false;
      }
    }

    return true;
  }

  // Connect to the battery sensor and run pre-dive checks, return true if successful
  bool DriverNode::connect_battery()
  {
    if (!read_battery()) {
      RCLCPP_ERROR(get_logger(), "can't read the battery, correct bus? member of i2c?");
      return false;
    }

    RCLCPP_INFO(get_logger(), "voltage is %g", driver_msg_.voltage);
    return !driver_msg_.low_battery;
  }

  // Connect to the leak sensor and run pre-dive checks, return true if successful
  bool DriverNode::connect_leak()
  {
    if (!read_leak()) {
      RCLCPP_ERROR(get_logger(), "can't read the leak sensor");
      return false;
    }

    RCLCPP_INFO(get_logger(), "leak status is %d", driver_msg_.leak_detected);
    return !driver_msg_.leak_detected;
  }

  // Connect to all hardware and run pre-dive checks, return true if we're ready to dive
  bool DriverNode::connect()
  {
    set_status(orca_msgs::msg::Driver::STATUS_NONE);
    if (!connect_battery() || !connect_controller() || !connect_leak()) {
      abort();
      return false;
    }

    RCLCPP_INFO(get_logger(), "hardware initialized, pre-dive checks passed");
    set_status(orca_msgs::msg::Driver::STATUS_OK);
    return true;
  }

  // Normal exit
  void DriverNode::disconnect()
  {
    RCLCPP_INFO(get_logger(), "normal exit");
    set_status(orca_msgs::msg::Driver::STATUS_NONE);
    all_stop();
    maestro_.disconnect();
  }

} // namespace orca_driver

//=============================================================================
// Main
//=============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_driver::DriverNode>();

  // Connect and run pre-dive checks
  if (node->connect()) {
    // Spin node
    rclcpp::spin(node);
  }

  // Disconnect
  node->disconnect();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
