#include "gazebo/gazebo.hh"
#include "gazebo/sensors/sensors.hh"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "orca_gazebo/orca_gazebo_util.hpp"
#include "orca_msgs/msg/barometer.hpp"

/* A very simple barometer sensor plugin for underwater robotics. Usage:
 *
 *    <gazebo reference="base_link">
 *      <sensor name="barometer_sensor" type="altimeter">
 *        <update_rate>60</update_rate>
 *        <plugin name="OrcaBarometerPlugin" filename="libOrcaBarometerPlugin.so">
 *          <baro_topic>/barometer</baro_topic>
 *          <pose_topic>/depth</pose_topic>
 *          <fluid_density>1029</fluid_density>
 *          <surface>10</surface>
 *        </plugin>
 *      </sensor>
 *    </gazebo>
 *
 *    <baro_topic> Topic for orca_msgs/Barometer messages. Default is /barometer.
 *    <pose_topic> Topic for geometry_msgs/PoseWithCovarianceStamped messages. Default is /depth.
 *    <fluid_density> Fluid density in kg/m^3. Default is 1029 (seawater), use 997 for freshwater.
 *    <surface> How far above z=0 the surface of the water is; used to calculate depth.
 */

namespace gazebo {

// TODO(Crystal): use <ros> tags w/ parameters to simplify the parameter blocks for Orca plugins

constexpr double SEAWATER_DENSITY = 1029;
constexpr double ATMOSPHERIC_PRESSURE = 101325;   // Pascals
constexpr double GRAVITY = 9.80665;               // m/s^2
constexpr double DEPTH_STDDEV = 0.01;             // m

class OrcaBarometerPlugin : public SensorPlugin
{
  // Our parent sensor is an altimeter
  sensors::AltimeterSensorPtr altimeter_;

  // Pointer to the Gazebo update event connection
  event::ConnectionPtr update_connection_;

  // Pointer to the GazeboROS node
  gazebo_ros::Node::SharedPtr node_;

  // ROS publishers
  rclcpp::Publisher<orca_msgs::msg::Barometer>::SharedPtr baro_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  double fluid_density_ = SEAWATER_DENSITY;   // Fluid density
  double surface_ = 10;                       // Altitude of water surface above the ground

public:

  // Called once when the plugin is loaded.
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
  {
    GZ_ASSERT(sensor != nullptr, "Sensor is null");
    GZ_ASSERT(sdf != nullptr, "SDF is null");

    // Get the GazeboROS node
    node_ = gazebo_ros::Node::Get(sdf);

    std::string baro_topic = "/barometer";
    std::string pose_topic = "/depth";

    std::cout << std::endl;
    std::cout << "ORCA BAROMETER PLUGIN PARAMETERS" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "Default baro topic: " << baro_topic << std::endl;
    std::cout << "Default pose topic: " << pose_topic << std::endl;
    std::cout << "Default fluid density: " << fluid_density_ << std::endl;
    std::cout << "Default surface: " << surface_ << std::endl;

    if (sdf->HasElement("baro_topic"))
    {
      baro_topic = sdf->GetElement("baro_topic")->Get<std::string>();
      std::cout << "Baro topic: " << baro_topic << std::endl;
    }
    baro_pub_ = node_->create_publisher<orca_msgs::msg::Barometer>(baro_topic, 1);

    if (sdf->HasElement("pose_topic"))
    {
      pose_topic = sdf->GetElement("pose_topic")->Get<std::string>();
      std::cout << "Pose topic: " << pose_topic << std::endl;
    }
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 1);

    // Get fluid density
    if (sdf->HasElement("fluid_density"))
    {
      fluid_density_ = sdf->GetElement("fluid_density")->Get<double>();
      std::cout << "Fluid density: " << fluid_density_ << std::endl;
    }

    // Get surface
    if (sdf->HasElement("surface"))
    {
      surface_ = sdf->GetElement("surface")->Get<double>();
      std::cout << "Surface: " << surface_ << std::endl;
    }

    // Get the parent sensor
    altimeter_ = std::dynamic_pointer_cast<sensors::AltimeterSensor>(sensor);

    // Listen to the update event
    update_connection_ = altimeter_->ConnectUpdated(std::bind(&OrcaBarometerPlugin::OnUpdate, this));

    // Activate the parent sensor
    altimeter_->SetActive(true);
  }

  // The update event is broadcast at the sensor frequency, roughly 60Hz
  void OnUpdate()
  {
    orca_msgs::msg::Barometer baro_msg;
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.frame_id = "odom";
    pose_msg.header.stamp = node_->now();
    pose_msg.pose.covariance[14] = DEPTH_STDDEV * DEPTH_STDDEV;

    // The altimeter sensor zeros out when it starts, so it must start at (0, 0, 0).
    double depth = orca_gazebo::gaussianKernel(surface_ - altimeter_->Altitude(), DEPTH_STDDEV);
    if (depth >= 0.0)
    {
      baro_msg.depth = depth;
      baro_msg.pressure = fluid_density_ * GRAVITY * depth + ATMOSPHERIC_PRESSURE; // Pascals
      baro_msg.temperature = 10; // Celsius

      pose_msg.pose.pose.position.z = -depth; // ENU
    }
    else
    {
      baro_msg.depth = 0;
      baro_msg.pressure = ATMOSPHERIC_PRESSURE;
      baro_msg.temperature = 20;

      pose_msg.pose.pose.position.z = 0;
    }

    pose_pub_->publish(pose_msg);
    baro_pub_->publish(baro_msg);
  }
};

GZ_REGISTER_SENSOR_PLUGIN(OrcaBarometerPlugin)

} // namespace gazebo