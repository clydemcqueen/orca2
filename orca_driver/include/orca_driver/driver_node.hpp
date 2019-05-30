#ifndef ORCA_DRIVER_H
#define ORCA_DRIVER_H

#include <string>
#include <vector>

#include "mraa/common.hpp"
#include "mraa/led.hpp"

#include "rclcpp/rclcpp.hpp"

#include "orca_driver/driver_context.hpp"
#include "orca_driver/maestro.hpp"
#include "orca_msgs/msg/battery.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/leak.hpp"

namespace orca_driver
{

  struct Thruster
  {
    int channel_;
    bool reverse_;
  };

  // DriverNode provides the interface between the Orca hardware and ROS.

  class DriverNode : public rclcpp::Node
  {
  private:

    // Parameters
    DriverContext cxt_;
    std::vector<Thruster> thrusters_;

    // State
    maestro::Maestro maestro_;
    orca_msgs::msg::Battery battery_msg_;
    orca_msgs::msg::Leak leak_msg_;

    // Subscriptions
    rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr spin_timer_;

    // Callbacks
    void control_callback(const orca_msgs::msg::Control::SharedPtr msg);

    // Publications
    rclcpp::Publisher<orca_msgs::msg::Battery>::SharedPtr battery_pub_;
    rclcpp::Publisher<orca_msgs::msg::Leak>::SharedPtr leak_pub_;

    // LEDs on the UP board
    // https://github.com/intel-iot-devkit/mraa/blob/master/examples/platform/up2-leds.cpp
    mraa::Led led_ready_{"green"};
    mraa::Led led_mission_{"yellow"};
    mraa::Led led_problem_{"red"};

    bool read_battery();

    bool read_leak();

    void spin_once();

    bool pre_dive();

    void all_stop();

    void abort();

  public:
    explicit DriverNode();

    ~DriverNode()
    {}; // Suppress default copy and move constructors

    bool connect();

    void disconnect();
  };

} // namespace orca_driver

#endif // ORCA_DRIVER_H
