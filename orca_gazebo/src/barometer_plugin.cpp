#include <random>

#include "gazebo/gazebo.hh"
#include "gazebo/sensors/sensors.hh"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_ros/node.hpp"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"

#include "orca_shared/model.hpp"
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

  constexpr double FRESHWATER_DENSITY = 997;          // Default water density
  constexpr double ATMOSPHERIC_PRESSURE = 101300;     // Default air pressure at the surface of the water
  constexpr int QUEUE_SIZE = 10;

  class OrcaBarometerPlugin : public SensorPlugin
  {
    // Simulate the barometer "in air" for a few seconds
    // This gives depth_node time to calibrate the barometer
    rclcpp::Time in_air_start_time_;
    const rclcpp::Duration TIME_IN_AIR{RCL_S_TO_NS(2)};
    bool in_air_{true};

    // Our parent sensor is an altimeter
    sensors::AltimeterSensorPtr altimeter_;

    // Pointer to the Gazebo update event connection
    event::ConnectionPtr update_connection_;

    // Pointer to the GazeboROS node
    gazebo_ros::Node::SharedPtr node_;

    // ROS publisher
    rclcpp::Publisher<orca_msgs::msg::Barometer>::SharedPtr baro_pub_;

    // Orca model
    orca::Model orca_model_;

    // Normal distribution
//    std::default_random_engine generator_;
//    std::normal_distribution<double> distribution_{0, orca::Model::DEPTH_STDDEV};

  public:

    // Called once when the plugin is loaded.
    void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
    {
      (void) update_connection_;

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
      baro_pub_ = node_->create_publisher<orca_msgs::msg::Barometer>(baro_topic, QUEUE_SIZE);

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

      // ROS sim time is only updated at 10Hz, and the camera sensor seems to publish frames
      // with timestamps aligned to this 10Hz clock rate. More investigation needed. TODO
      // In the meanwhile, run everything on the wall clock.

//      rclcpp::Time msg_time = node_->now();
      auto t = std::chrono::high_resolution_clock::now();
      rclcpp::Time msg_time{t.time_since_epoch().count(), RCL_ROS_TIME};

      // TODO get from urdf or tf tree
      static const double baro_link_to_base_link_z = -0.05;

      if (node_->count_subscribers(baro_pub_->get_topic_name()) > 0) {
        orca_msgs::msg::Barometer baro_msg;
        baro_msg.header.frame_id = "map";
        baro_msg.header.stamp = msg_time;

        // Start the "in air" time
        if (in_air_start_time_.nanoseconds() == 0) {
          in_air_start_time_ = msg_time;
          std::cout << "barometer is in air" << std::endl;
        }

        // Move to the water after a few seconds
        if (in_air_ && msg_time > (in_air_start_time_ + TIME_IN_AIR)) {
          in_air_ = false;
          std::cout << "barometer is in water" << std::endl;
        }

        /************************************************
         * A bit tricky...
         *
         * The model is injected at {0, 0, 0}, which means base_link is {0, 0, 0}.
         * The altimeter will calibrate right away, but since baro_link is 5cm higher than base_link,
         * calls to Altitude() will be too low by 5cm. Compensate.
         */

        // TODO re-enable noise
        double baro_link_z = altimeter_->Altitude() /* + distribution_(generator_) */ - baro_link_to_base_link_z;
//        std::cout << "Altitude() " << altimeter_->Altitude() << "baro_link_z " << baro_link_z << std::endl;

        if (!in_air_ && baro_link_z < 0.0) {
          baro_msg.pressure = orca_model_.z_to_pressure(ATMOSPHERIC_PRESSURE, baro_link_z); // Pascals
          baro_msg.temperature = 10; // Celsius
        } else {
          baro_msg.pressure = ATMOSPHERIC_PRESSURE;
          baro_msg.temperature = 20;
        }

        baro_pub_->publish(baro_msg);
      }
    }
  };

  GZ_REGISTER_SENSOR_PLUGIN(OrcaBarometerPlugin)

} // namespace gazebo