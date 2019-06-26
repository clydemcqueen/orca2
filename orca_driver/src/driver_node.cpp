#include "orca_driver/driver_node.hpp"

namespace orca_driver
{

  const rclcpp::Duration CONTROL_TIMEOUT{RCL_S_TO_NS(3)}; // All-stop if control messages stop

  bool valid(const rclcpp::Time &t)
  {
    return t.nanoseconds() > 0;
  }

  //=============================================================================
  // DriverNode
  //=============================================================================

  DriverNode::DriverNode() :
    Node{"orca_driver"}
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

    // Publish battery and leak messages
    battery_pub_ = create_publisher<orca_msgs::msg::Battery>("battery", 1);
    leak_pub_ = create_publisher<orca_msgs::msg::Leak>("leak", 1);

    // Subscribe to control messages
    using std::placeholders::_1;
    auto control_cb = std::bind(&DriverNode::control_callback, this, _1);
    control_sub_ = create_subscription<orca_msgs::msg::Control>("control", 1, control_cb);

    // Spin timer
    using namespace std::chrono_literals;
    spin_timer_ = create_wall_timer(500ms, std::bind(&DriverNode::timer_callback, this));
  }

  void DriverNode::validate_parameters()
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    DRIVER_NODE_ALL_PARAMS
  }

  void DriverNode::set_status(Status status)
  {
    if (status != status_) {
      status_ = status;

#if 0
      led_ready_.setBrightness(0);
      led_mission_.setBrightness(0);
      led_problem_.setBrightness(0);

      switch (status_) {
        case Status::ready:
          led_ready_.setBrightness(led_ready_.readMaxBrightness() / 2);
          break;
        case Status::mission:
          led_mission_.setBrightness(led_mission_.readMaxBrightness() / 2);
          break;
        case Status::problem:
          led_problem_.setBrightness(led_problem_.readMaxBrightness() / 2);
          break;
        default:
          break;
      }
#endif
    }
  }

  void DriverNode::control_callback(const orca_msgs::msg::Control::SharedPtr msg)
  {
    if (!valid(control_msg_time_)) {
      RCLCPP_INFO(get_logger(), "receiving control messages");
    }

    control_msg_time_ = now();

    set_status(msg->mode >= msg->KEEP_STATION ? Status::mission : Status::ready);

    if (maestro_.ready()) {
      if (!maestro_.setPWM(static_cast<uint8_t>(cxt_.tilt_channel_), msg->camera_tilt_pwm)) {
        RCLCPP_ERROR(get_logger(), "failed to set camera tilt");
      }
      if (!maestro_.setPWM(static_cast<uint8_t>(cxt_.lights_channel_), msg->brightness_pwm)) {
        RCLCPP_ERROR(get_logger(), "failed to set brightness");
      }

      for (int i = 0; i < thrusters_.size(); ++i) {
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

    if (!read_battery() || !read_leak() || battery_msg_.low_battery || leak_msg_.leak_detected) {
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

    battery_pub_->publish(battery_msg_);
    leak_pub_->publish(leak_msg_);
  }

  // Read battery sensor, return true if successful
  bool DriverNode::read_battery()
  {
    battery_msg_.header.stamp = now();

    double value = 0.0;
    if (maestro_.ready() && maestro_.getAnalog(static_cast<uint8_t>(cxt_.voltage_channel_), value)) {
      battery_msg_.voltage = value * cxt_.voltage_multiplier_;
      battery_msg_.low_battery = static_cast<uint8_t>(battery_msg_.voltage < cxt_.voltage_min_);
      if (battery_msg_.low_battery) {
        RCLCPP_ERROR(get_logger(), "battery voltage %g is below minimum %g", battery_msg_.voltage, cxt_.voltage_min_);
      }
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "can't read battery");
      battery_msg_.voltage = 0;
      battery_msg_.low_battery = 1;
      return false;
    }
  }

  // Read leak sensor, return true if successful
  bool DriverNode::read_leak()
  {
    leak_msg_.header.stamp = now();

    bool value = 0.0;
    if (maestro_.ready() && maestro_.getDigital(static_cast<uint8_t>(cxt_.leak_channel_), value)) {
      leak_msg_.leak_detected = static_cast<uint8_t>(value);
      if (leak_msg_.leak_detected) {
        RCLCPP_ERROR(get_logger(), "leak detected");
      }
      return true;
    } else {
      RCLCPP_ERROR(get_logger(), "can't read leak sensor");
      leak_msg_.leak_detected = 1;
      return false;
    }
  }

  // Run a bunch of pre-dive checks, return true if everything looks good
  bool DriverNode::pre_dive()
  {
    RCLCPP_INFO(get_logger(), "running pre-dive checks...");

    if (!read_battery() || !read_leak()) {
      maestro_.disconnect();
      return false;
    }

    RCLCPP_INFO(get_logger(), "voltage is %g, leak status is %d", battery_msg_.voltage, leak_msg_.leak_detected);

    if (leak_msg_.leak_detected) {
      maestro_.disconnect();
      return false;
    }

    if (battery_msg_.low_battery) {
      maestro_.disconnect();
      return false;
    }

    // When the Maestro boots, it should set all thruster channels to 1500.
    // But on a system restart it might be a bad state. Force an all-stop.
    all_stop();

    // Check to see that all thrusters are stopped.
    for (int i = 0; i < thrusters_.size(); ++i) {
      uint16_t value = 0;
      maestro_.getPWM(static_cast<uint8_t>(thrusters_[i].channel_), value);
      RCLCPP_INFO(get_logger(), "thruster %d is set at %d", i + 1, value);
      if (value != orca_msgs::msg::Control::THRUST_STOP) {
        RCLCPP_ERROR(get_logger(), "thruster %d didn't initialize properly (and possibly others)", i + 1);
        maestro_.disconnect();
        return false;
      }
    }

    RCLCPP_INFO(get_logger(), "pre-dive checks passed");
    set_status(Status::ready);
    return true;
  }

  // Stop all motion
  void DriverNode::all_stop()
  {
    RCLCPP_INFO(get_logger(), "all stop");
    if (maestro_.ready()) {
      for (int i = 0; i < thrusters_.size(); ++i) {
        maestro_.setPWM(static_cast<uint8_t>(thrusters_[i].channel_), orca_msgs::msg::Control::THRUST_STOP);
      }
    }
  }

  // Abnormal exit
  void DriverNode::abort()
  {
    RCLCPP_ERROR(get_logger(), "aborting dive");
    set_status(Status::problem);
    all_stop();
    maestro_.disconnect();
  }

  // Connect to Maestro and run pre-dive checks, return true if we're ready to dive
  bool DriverNode::connect()
  {
    set_status(Status::none);
    std::string port = cxt_.maestro_port_;
    RCLCPP_INFO(get_logger(), "opening port %s...", port.c_str());
    maestro_.connect(port);
    if (!maestro_.ready()) {
      RCLCPP_ERROR(get_logger(), "can't open port %s, connected? member of dialout?", port.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "port %s open", port.c_str());

    return pre_dive();
  }

  // Normal exit
  void DriverNode::disconnect()
  {
    RCLCPP_INFO(get_logger(), "normal exit");
    set_status(Status::none);
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
