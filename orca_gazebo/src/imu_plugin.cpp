// Copyright (c) 2020, Clyde McQueen.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <string>

#include "gazebo/gazebo.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo_ros/node.hpp"
#include "orca_gazebo/orca_gazebo_util.hpp"
#include "orca_msgs/msg/barometer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

/* IMU sensor plugin. Supports independent Gaussian noise for magnetometers, gyros and
 * accelerometers. Usage:
 *
 *    <gazebo reference="base_link">
 *      <gravity>true</gravity>
 *      <sensor name="imu_sensor" type="imu">
 *        <update_rate>125</update_rate>
 *        <plugin name="OrcaIMUPlugin" filename="libOrcaIMUPlugin.so" />
 *      </sensor>
 *    </gazebo>
 *
 * Publishes sensor_msgs::Imu messages on /imu/data.
 */

namespace gazebo
{

// TODO(clyde): publish sensor_msgs::MagneticField messages on /imu/mag
// TODO(clyde): remove orientation from sensor_msgs::Imu messages, publish on /imu/data_raw instead
// TODO(clyde): allow use of rotated links like imu_link

// https://github.com/ros-drivers/phidgets_drivers/blob/kinetic/phidgets_imu/src/imu_ros_i.cpp
constexpr double G = 9.80665;
constexpr double ACCEL_STDDEV = 300.0 * 1e-6 * G;
constexpr double GYRO_STDDEV = 0.02 * (M_PI / 180.0);
constexpr double MAG_STDDEV = 0.095 * (M_PI / 180.0);
constexpr double ORIENTATION_STDDEV = MAG_STDDEV;    // Temporary hack
constexpr int QUEUE_SIZE = 10;

const char IMU_TOPIC[] = "/imu/data";

class OrcaIMUPlugin : public SensorPlugin
{
  // Our parent sensor is an imu sensor
  sensors::ImuSensorPtr sensor_;

  // Pointer to the Gazebo update event connection
  event::ConnectionPtr update_connection_;

  // Pointer to the GazeboROS node
  gazebo_ros::Node::SharedPtr node_;

  // ROS publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // ROS message
  sensor_msgs::msg::Imu imu_msg_;

public:
  // Called once when the plugin is loaded.
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override
  {
    // Get the GazeboROS node
    node_ = gazebo_ros::Node::Get(sdf);

    // Prepare to publish
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, QUEUE_SIZE);
    imu_msg_.header.frame_id = "base_link";
    imu_msg_.linear_acceleration_covariance[0] = ACCEL_STDDEV * ACCEL_STDDEV;
    imu_msg_.linear_acceleration_covariance[4] = ACCEL_STDDEV * ACCEL_STDDEV;
    imu_msg_.linear_acceleration_covariance[8] = ACCEL_STDDEV * ACCEL_STDDEV;
    imu_msg_.angular_velocity_covariance[0] = GYRO_STDDEV * GYRO_STDDEV;
    imu_msg_.angular_velocity_covariance[4] = GYRO_STDDEV * GYRO_STDDEV;
    imu_msg_.angular_velocity_covariance[8] = GYRO_STDDEV * GYRO_STDDEV;
    imu_msg_.orientation_covariance[0] = ORIENTATION_STDDEV * ORIENTATION_STDDEV;
    imu_msg_.orientation_covariance[4] = ORIENTATION_STDDEV * ORIENTATION_STDDEV;
    imu_msg_.orientation_covariance[8] = ORIENTATION_STDDEV * ORIENTATION_STDDEV;

    // Get the parent sensor
    sensor_ = std::dynamic_pointer_cast<sensors::ImuSensor>(sensor);

    // Listen to the update event
    update_connection_ = sensor_->ConnectUpdated(std::bind(&OrcaIMUPlugin::OnUpdate, this));

    // Activate the parent sensor
    sensor_->SetActive(true);
  }

  // The update event is broadcast at the sensor frequency
  void OnUpdate()
  {
    // Only publish if there are subscribers
    if (node_->count_subscribers(IMU_TOPIC) > 0) {
      // Don't publish bogus time stamps
      if (node_->now().nanoseconds() <= 0) {
        return;
      }

      // TODO(clyde): use find & use the equivalent of LastMeasurementTime(), see barometer plugin
      imu_msg_.header.stamp = node_->now();

      // Get accel and gyro readings in the sensor frame (base_link)
      ignition::math::Vector3d linear_acceleration = sensor_->LinearAcceleration(true);
      ignition::math::Vector3d angular_velocity = sensor_->AngularVelocity(true);

      // Get orientation in the reference frame -- set on boot to the world frame (odom)
      ignition::math::Quaterniond orientation = sensor_->Orientation();

      // TODO(clyde): turn orientation into a magnetometer reading -- tricky

      // Add noise
      orca_gazebo::addNoise(ACCEL_STDDEV, linear_acceleration);
      orca_gazebo::addNoise(GYRO_STDDEV, angular_velocity);
      orca_gazebo::addNoise(ORIENTATION_STDDEV, orientation);

      // Copy to message
      orca_gazebo::ignition2msg(linear_acceleration, imu_msg_.linear_acceleration);
      orca_gazebo::ignition2msg(angular_velocity, imu_msg_.angular_velocity);
      orca_gazebo::ignition2msg(orientation, imu_msg_.orientation);

      imu_pub_->publish(imu_msg_);
    }
  }
};

GZ_REGISTER_SENSOR_PLUGIN(OrcaIMUPlugin)

}  // namespace gazebo
