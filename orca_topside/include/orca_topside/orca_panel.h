#ifndef ORCA_PANEL_H
#define ORCA_PANEL_H

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

#include "orca_msgs/msg/barometer.hpp"
#include "orca_msgs/msg/battery.hpp"
#include "orca_msgs/msg/control.hpp"
#include "orca_msgs/msg/leak.hpp"
#include "orca_msgs/msg/proc.hpp"

class QLabel;

namespace orca_topside {

class OrcaPanel;

// Proxy node: subscribe to a bunch of ROS topics, and update the OrcaPanel
class ProxyNode: public rclcpp::Node
{
public:
  explicit ProxyNode(OrcaPanel *panel);
  ~ProxyNode();

protected:
  // The panel to update
  OrcaPanel *panel_;

  rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Subscription<orca_msgs::msg::Battery>::SharedPtr battery_sub_;
  rclcpp::Subscription<orca_msgs::msg::Control>::SharedPtr control_sub_;
  rclcpp::Subscription<orca_msgs::msg::Leak>::SharedPtr leak_sub_;
  rclcpp::Subscription<orca_msgs::msg::Proc>::SharedPtr proc_sub_;

  void baroCallback(const orca_msgs::msg::Barometer::SharedPtr msg);
  void batteryCallback(const orca_msgs::msg::Battery::SharedPtr msg);
  void controlCallback(const orca_msgs::msg::Control::SharedPtr msg);
  void leakCallback(const orca_msgs::msg::Leak::SharedPtr msg);
  void procCallback(const orca_msgs::msg::Proc::SharedPtr msg);
};

// Orca panel: display a bunch of interesting information
class OrcaPanel: public rviz_common::Panel
{
public:
  OrcaPanel(QWidget* parent = 0);
  ~OrcaPanel();

protected:
  // Pointer to a proxy node that will subscribe to ROS topics
  friend class ProxyNode;
  std::shared_ptr<ProxyNode> node_;

  QLabel* battery_viewer_;
  QLabel* camera_tilt_viewer_;
  QLabel* depth_viewer_;
  QLabel* leak_viewer_;
  QLabel* lights_viewer_;
  QLabel* mode_viewer_;
  QLabel* proc_viewer_;
  QLabel* temperature_viewer_;
  QLabel* yaw_viewer_;

  QPalette ok_palette_;
  QPalette alert_palette_;
  QPalette danger_palette_;
};

} // namespace orca_topside

#endif // ORCA_PANEL_H
