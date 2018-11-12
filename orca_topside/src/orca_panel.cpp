//#include "tf2_ros/transform_listener.h"
//#include "geometry_msgs/msg/transform_stamped.hpp"

//#include "rviz_common/frame_manager.hpp"
#include "rviz_common/logging.hpp"

//#include "qt5/QtCore/QtCore"
#include "QVBoxLayout"
#include "QLabel"

//#include "orca_base/orca_pwm.h"
#include "orca_topside/orca_panel.h"

namespace orca_topside {

ProxyNode::ProxyNode(OrcaPanel *panel) :
  Node("orca_panel"),
  panel_(panel)
{
  using std::placeholders::_1;
  baro_sub_ = create_subscription<orca_msgs::msg::Barometer>("/barometer", std::bind(&ProxyNode::baroCallback, this, _1));
  battery_sub_ = create_subscription<orca_msgs::msg::Battery>("/orca_driver/battery", std::bind(&ProxyNode::batteryCallback, this, _1));
  control_sub_ = create_subscription<orca_msgs::msg::Control>("/orca_base/control", std::bind(&ProxyNode::controlCallback, this, _1));
  leak_sub_ = create_subscription<orca_msgs::msg::Leak>("/orca_driver/leak", std::bind(&ProxyNode::leakCallback, this, _1));
  proc_sub_ = create_subscription<orca_msgs::msg::Proc>("/proc", std::bind(&ProxyNode::procCallback, this, _1));
}

ProxyNode::~ProxyNode()
{
  // Unsubscribe from everything
  baro_sub_ = nullptr;
  battery_sub_ = nullptr;
  control_sub_ = nullptr;
  leak_sub_ = nullptr;
  proc_sub_ = nullptr;

  panel_ = nullptr;
}


// TODO redo this w/ odometry messages
#if 0
// Convert yaw (magnetic North, East is 0.0, right-handed, radians)
// to compass heading (true North, East is 90.0, left-handed, degrees)
constexpr const double yaw2heading(double yaw)
{
  constexpr double declination = 15.44; // Seattle declination is 15.44 E
  return 90.0 + declination - qRadiansToDegrees(yaw);
}

void ProxyNode::timerCallback()
{
  tf2_ros::TransformListener *tfListener = panel_->get context_->getTFClient();
  if (tfListener != nullptr && tfListener->canTransform("odom", "base_link", rclcpp::Time(0)))
  {
    try
    {
      // Get odometry data
      geometry_msgs::msg::TransformStamped transform;
      tfListener->lookupTransform("odom", "base_link", rclcpp::Time(0), transform);

      // Odom depth is based on the Bar30 reading, but it's zeroed out on boot
      double depth = -transform.getOrigin().getZ();

      QString depth_string = QString("Depth %1m").arg(depth, -1, 'f', 2);
      depth_viewer_->setText(depth_string);

      // Pull out the yaw and compute a compass heading
      tf::Quaternion orientation = transform.getRotation();
      double roll, pitch, yaw;
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      double heading = yaw2heading(yaw);

      QString heading_string = QString("Heading %1°").arg(heading, -1, 'f', 0);
      yaw_viewer_->setText(heading_string);
    }
    catch (const std::exception &ex)
    {
      RVIZ_COMMON_LOG_ERROR_STREAM("Exception trying to get odom to base_link transform: " << ex.what());
    }
  }
}
#endif

void ProxyNode::baroCallback(const orca_msgs::msg::Barometer::SharedPtr msg)
{
  // Prefer tf data over raw pressure reading
  panel_->temperature_viewer_->setText(QString("Water temp %1°").arg(msg->temperature));
}

void ProxyNode::batteryCallback(const orca_msgs::msg::Battery::SharedPtr msg)
{
  panel_->battery_viewer_->setText(QString("Battery %1V").arg(msg->voltage, -1, 'f', 2));
  panel_->battery_viewer_->setPalette(
    msg->voltage > 15 ? panel_->ok_palette_ : (msg->voltage > 14 ? panel_->alert_palette_ : panel_->danger_palette_));
}

void ProxyNode::controlCallback(const orca_msgs::msg::Control::SharedPtr msg)
{
  //int camera_tilt = orca_base::pwm_to_tilt(msg->camera_tilt_pwm);
  //int brightness = orca_base::pwm_to_brightness(msg->brightness_pwm);
  //panel_->camera_tilt_viewer_->setText(QString("Camera tilt %1°").arg(camera_tilt));
  //panel_->lights_viewer_->setText(QString("Lights %1\%").arg(brightness));

  switch (msg->mode)
  {
    case orca_msgs::msg::Control::DISARMED:
      panel_->mode_viewer_->setText("Disarmed");
      break;
    case orca_msgs::msg::Control::MANUAL:
      panel_->mode_viewer_->setText("Manual control");
      break;
    case orca_msgs::msg::Control::HOLD_H:
      panel_->mode_viewer_->setText("Hold heading");
      break;
    case orca_msgs::msg::Control::HOLD_D:
      panel_->mode_viewer_->setText("Hold depth");
      break;
    case orca_msgs::msg::Control::HOLD_HD:
      panel_->mode_viewer_->setText("Hold heading and depth");
      break;
    case orca_msgs::msg::Control::MISSION:
      panel_->mode_viewer_->setText("Running mission");
      break;
    case orca_msgs::msg::Control::SOS:
      panel_->mode_viewer_->setText("SOS!");
      break;
    default:
      panel_->mode_viewer_->setText("ERROR: unknown mode");
      break;
  }
}

void ProxyNode::leakCallback(const orca_msgs::msg::Leak::SharedPtr msg)
{
  panel_->leak_viewer_->setText(QString(msg->leak_detected ? "LEAK LEAK LEAK LEAK" : "No leak"));
  panel_->leak_viewer_->setPalette(msg->leak_detected ? panel_->danger_palette_ : panel_->ok_palette_);
}

void ProxyNode::procCallback(const orca_msgs::msg::Proc::SharedPtr msg)
{
  panel_->proc_viewer_->setText(QString("Processor temp %1°").arg(msg->cpu_temp, -1, 'f', 1));
  panel_->proc_viewer_->setPalette(
    msg->cpu_temp < 70 ? panel_->ok_palette_ : (msg->cpu_temp < 80 ? panel_->alert_palette_ : panel_->danger_palette_));
}

OrcaPanel::OrcaPanel(QWidget *parent) :
  rviz_common::Panel(parent)
{
  RVIZ_COMMON_LOG_DEBUG("Constructing OrcaPanel");

  node_ = std::make_shared<ProxyNode>(this);

  ok_palette_ = palette();
  alert_palette_ = palette();
  alert_palette_.setColor(QPalette::Background, Qt::yellow);
  danger_palette_ = palette();
  danger_palette_.setColor(QPalette::Background, Qt::red);
  danger_palette_.setColor(QPalette::Foreground, Qt::white);

  QVBoxLayout *layout = new QVBoxLayout;
  QFont big;
  big.setPointSize(16);

  mode_viewer_ = new QLabel("Mode unknown");
  layout->addWidget(mode_viewer_);

  yaw_viewer_ = new QLabel("Heading unknown");
  yaw_viewer_->setFont(big);
  layout->addWidget(yaw_viewer_);

  depth_viewer_ = new QLabel("Depth unknown");
  depth_viewer_->setFont(big);
  layout->addWidget(depth_viewer_);

  battery_viewer_ = new QLabel("Battery unknown");
  battery_viewer_->setFont(big);
  battery_viewer_->setAutoFillBackground(true);
  battery_viewer_->setPalette(ok_palette_);
  layout->addWidget(battery_viewer_);

  leak_viewer_ = new QLabel("Leak unknown");
  leak_viewer_->setAutoFillBackground(true);
  leak_viewer_->setPalette(ok_palette_);
  layout->addWidget(leak_viewer_);

  proc_viewer_ = new QLabel("Processor temp unknown");
  proc_viewer_->setAutoFillBackground(true);
  proc_viewer_->setPalette(ok_palette_);
  layout->addWidget(proc_viewer_);

  camera_tilt_viewer_ = new QLabel("Camera tilt unknown");
  layout->addWidget(camera_tilt_viewer_);

  lights_viewer_ = new QLabel("Lights unknown");
  layout->addWidget(lights_viewer_);

  temperature_viewer_ = new QLabel("Water temp unknown");
  layout->addWidget(temperature_viewer_);

  layout->addStretch();
  setLayout(layout);

  RVIZ_COMMON_LOG_DEBUG("OrcaPanel running");

  // TODO need to spin the node
}

OrcaPanel::~OrcaPanel()
{
  node_ = nullptr;
}

} // namespace orca_topside

// Tell pluginlib about this class
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orca_topside::OrcaPanel, rviz_common::Panel)
