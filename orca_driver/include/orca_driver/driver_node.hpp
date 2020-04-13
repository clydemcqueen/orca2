#ifndef ORCA_DRIVER_H
#define ORCA_DRIVER_H

#include <string>
#include <vector>

#include "mraa/common.hpp"
#include "mraa/led.hpp"

#include "rclcpp/rclcpp.hpp"

#include "orca_driver/driver_context.hpp"
#include "orca_driver/maestro.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/driver.hpp"

namespace orca_driver
{

  // LEDs on the UP board
  // https://github.com/intel-iot-devkit/mraa/blob/master/examples/platform/up2-leds.cpp
#define LEDS
#ifdef LEDS
#define LED_READY_ON() led_ready_.setBrightness(led_ready_.readMaxBrightness() / 2)
#define LED_MISSION_ON() led_mission_.setBrightness(led_mission_.readMaxBrightness() / 2)
#define LED_PROBLEM_ON() led_problem_.setBrightness(led_problem_.readMaxBrightness() / 2)
#define LED_READY_OFF() led_ready_.setBrightness(0)
#define LED_MISSION_OFF() led_mission_.setBrightness(0)
#define LED_PROBLEM_OFF() led_problem_.setBrightness(0)
#else
#define LED_READY_ON()
#define LED_MISSION_ON()
#define LED_PROBLEM_ON()
#define LED_READY_OFF()
#define LED_MISSION_OFF()
#define LED_PROBLEM_OFF()
#endif

  struct Thruster
  {
    int channel_;
    bool reverse_;
  };

  // DriverNode provides the interface between the Orca hardware and ROS.

  class DriverNode : public rclcpp::Node
  {
    // Parameters
    DriverContext cxt_;
    std::vector<Thruster> thrusters_;

    // Timeout, set by parameter
    rclcpp::Duration control_timeout_{0};

    // State
    maestro::Maestro maestro_;
    orca_msgs::msg::Driver driver_msg_;

    // Control message state
    rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
    rclcpp::Time control_msg_time_;

    // Timer
    rclcpp::TimerBase::SharedPtr spin_timer_;

    // Publication
    rclcpp::Publisher<orca_msgs::msg::Driver>::SharedPtr driver_pub_;

#ifdef LEDS
    mraa::Led led_ready_{"yellow"};
    mraa::Led led_mission_{"green"};
    mraa::Led led_problem_{"red"};
#endif

    void validate_parameters();

    void set_status(uint8_t status);

    void control_callback(const orca_msgs::msg::Control::SharedPtr msg);

    void timer_callback();

    bool read_battery();

    bool read_leak();

    bool connect_controller();

    void all_stop();

    void abort();

  public:
    explicit DriverNode();

    ~DriverNode() override;
  };

} // namespace orca_driver

#endif // ORCA_DRIVER_H
