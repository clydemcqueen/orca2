#include <random>

#include "gazebo/gazebo.hh"
#include "gazebo/sensors/sensors.hh"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"

#include "orca_base/model.hpp"
#include "orca_gazebo/orca_gazebo_util.hpp"
#include "orca_msgs/msg/barometer.hpp"

/* A very simple barometer sensor plugin for underwater robotics. Usage:
 *
 *    <gazebo reference="base_link">
 *      <sensor name="barometer_sensor" type="altimeter">
 *        <update_rate>60</update_rate>
 *        <plugin name="OrcaBarometerPlugin" filename="libOrcaBarometerPlugin.so">
 *          <baro_topic>/barometer</baro_topic>
 *          <fluid_density>997</fluid_density>
 *        </plugin>
 *      </sensor>
 *    </gazebo>
 *
 *    <baro_topic> Topic for orca_msgs/Barometer messages. Default is /barometer.
 *    <fluid_density> Fluid density in kg/m^3. Default is 997 (freshwater), use 1029 for seawater.
 */

namespace gazebo
{

  constexpr double FRESHWATER_DENSITY = 997;

  class OrcaBarometerPlugin : public SensorPlugin
  {
    const rclcpp::Time IN_WATER{RCL_S_TO_NS(1), RCL_ROS_TIME};  // Simulate barometer "in air" for 1s

    // Our parent sensor is an altimeter
    sensors::AltimeterSensorPtr altimeter_;

    // Pointer to the Gazebo update event connection
    event::ConnectionPtr update_connection_;

    // Pointer to the GazeboROS node
    gazebo_ros::Node::SharedPtr node_;

    // ROS publisher
    rclcpp::Publisher<orca_msgs::msg::Barometer>::SharedPtr baro_pub_;

    // Orca model
    orca_base::Model orca_model_;

    // Normal distribution
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_{0, orca_base::Model::DEPTH_STDDEV};

  public:

    // Called once when the plugin is loaded.
    void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
    {
      GZ_ASSERT(sensor != nullptr, "Sensor is null");
      GZ_ASSERT(sdf != nullptr, "SDF is null");

      // Get the GazeboROS node
      node_ = gazebo_ros::Node::Get(sdf);

      // Default parameters
      std::string baro_topic = "/barometer";
      double fluid_density = FRESHWATER_DENSITY;

      if (sdf->HasElement("baro_topic")) {
        baro_topic = sdf->GetElement("baro_topic")->Get<std::string>();
      }
      RCLCPP_INFO(node_->get_logger(), "baro topic: %s", baro_topic.c_str());
      baro_pub_ = node_->create_publisher<orca_msgs::msg::Barometer>(baro_topic, 1);

      if (sdf->HasElement("fluid_density")) {
        fluid_density = sdf->GetElement("fluid_density")->Get<double>();
      }
      RCLCPP_INFO(node_->get_logger(), "fluid density: %g", fluid_density);

      // Initialize model from parameters
      orca_model_.fluid_density_ = fluid_density;

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

      // TODO pull these from the URDF
      static const double z_top_to_baro_link = -0.05;
      static const double z_baro_link_to_base_link = -0.085;

      rclcpp::Time msg_time = node_->now();

      if (node_->count_subscribers(baro_pub_->get_topic_name()) > 0) {
        orca_msgs::msg::Barometer baro_msg;
        baro_msg.header.frame_id = "map";
        baro_msg.header.stamp = msg_time;

        // The altimeter sensor zeros out when it starts, so it must start at (0, 0, 0).
        double z = altimeter_->Altitude() + distribution_(generator_) - z_baro_link_to_base_link;

        if (msg_time > IN_WATER && z < 0.0) {
          baro_msg.pressure = orca_model_.z_to_pressure(z); // Pascals
          baro_msg.temperature = 10; // Celsius
        } else {
          baro_msg.pressure = orca_base::Model::ATMOSPHERIC_PRESSURE;
          baro_msg.temperature = 20;
        }

        baro_pub_->publish(baro_msg);
      }
    }
  };

  GZ_REGISTER_SENSOR_PLUGIN(OrcaBarometerPlugin)

} // namespace gazebo