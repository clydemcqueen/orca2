#include <random>

#include "gazebo/gazebo.hh"
#include "gazebo/sensors/sensors.hh"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
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
 *          <fluid_density>1029</fluid_density>
 *        </plugin>
 *      </sensor>
 *    </gazebo>
 *
 *    <baro_topic> Topic for orca_msgs/Barometer messages. Default is /barometer.
 *    <fluid_density> Fluid density in kg/m^3. Default is 1029 (seawater), use 997 for freshwater.
 */

namespace gazebo
{

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

    // ROS publisher
    rclcpp::Publisher<orca_msgs::msg::Barometer>::SharedPtr baro_pub_;

    double fluid_density_ = SEAWATER_DENSITY;   // Fluid density

    // Normal distribution
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_{0, DEPTH_STDDEV};

  public:

    // Called once when the plugin is loaded.
    void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
    {
      GZ_ASSERT(sensor != nullptr, "Sensor is null");
      GZ_ASSERT(sdf != nullptr, "SDF is null");

      // Get the GazeboROS node
      node_ = gazebo_ros::Node::Get(sdf);

      std::string baro_topic = "/barometer";

      if (sdf->HasElement("baro_topic")) {
        baro_topic = sdf->GetElement("baro_topic")->Get<std::string>();
      }
      RCLCPP_INFO(node_->get_logger(), "baro topic: %s", baro_topic.c_str());
      baro_pub_ = node_->create_publisher<orca_msgs::msg::Barometer>(baro_topic, 1);

      if (sdf->HasElement("fluid_density")) {
        fluid_density_ = sdf->GetElement("fluid_density")->Get<double>();
      }
      RCLCPP_INFO(node_->get_logger(), "fluid density: %g", fluid_density_);

      // Get the parent sensor
      altimeter_ = std::dynamic_pointer_cast<sensors::AltimeterSensor>(sensor);

      // Listen to the update event
      update_connection_ = altimeter_->ConnectUpdated(std::bind(&OrcaBarometerPlugin::OnUpdate, this));

      // Activate the parent sensor
      altimeter_->SetActive(true);
    }

    // The update event is broadcast at the sensor frequency, see xacro/urdf/sdf file
    void OnUpdate()
    {
      // There doesn't seem to be a good way to get consistent timestamps between
      // the Altimeter sensor and the Camera sensor. More investigation needed. TODO

//      builtin_interfaces::msg::Time now = node_->now();
//      builtin_interfaces::msg::Time update = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
//        altimeter_->LastUpdateTime());
//      builtin_interfaces::msg::Time measurement = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
//        altimeter_->LastMeasurementTime());
//      std::cout
//        << "now sec=" << now.sec << ", nsec=" << now.nanosec
//        << ", update sec=" << update.sec << ", nsec=" << update.nanosec
//        << ", measurement sec=" << measurement.sec << ", nsec=" << measurement.nanosec
//        << std::endl;

      builtin_interfaces::msg::Time msg_time = node_->now();

      // The altimeter sensor zeros out when it starts, so it must start at (0, 0, 0).
      double z = altimeter_->Altitude() + distribution_(generator_);

      if (node_->count_subscribers(baro_pub_->get_topic_name()) > 0) {
        orca_msgs::msg::Barometer baro_msg;
        baro_msg.header.frame_id = "map";
        baro_msg.header.stamp = msg_time;
        baro_msg.z_variance = DEPTH_STDDEV * DEPTH_STDDEV;

        if (z < 0.0) {
          baro_msg.z = z;
          baro_msg.pressure = fluid_density_ * GRAVITY * -z + ATMOSPHERIC_PRESSURE; // Pascals
          baro_msg.temperature = 10; // Celsius
        } else {
          baro_msg.z = 0;
          baro_msg.pressure = ATMOSPHERIC_PRESSURE;
          baro_msg.temperature = 20;
        }

        baro_pub_->publish(baro_msg);
      }
    }
  };

  GZ_REGISTER_SENSOR_PLUGIN(OrcaBarometerPlugin)

} // namespace gazebo