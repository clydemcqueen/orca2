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

#include <memory>
#include <string>

#include "orca_msgs/msg/barometer.hpp"
#include "orca_shared/monotonic.hpp"
#include "rclcpp/node.hpp"
#include "ukf/ukf.hpp"

namespace orca_filter
{

//=============================================================================
// A simple barometer filter, for use during ROV mode.
// There's no control input.
//=============================================================================

constexpr int STATE_DIM = 3;                // [p, vp, ap]T
constexpr int MEASUREMENT_DIM = 1;          // [p]
constexpr int CONTROL_DIM = 0;              // No control input
constexpr double PROCESS_NOISE = 20000.0;   // Measurement var is ~40k, process var should be close

constexpr int QUEUE_SIZE = 10;

class BaroFilterNode : public rclcpp::Node
{
  // Kalman filter
  ukf::UnscentedKalmanFilter filter_{STATE_DIM, 0.001, 2.0, 0};

  // Control
  Eigen::VectorXd u_ = Eigen::VectorXd::Zero(CONTROL_DIM);

  // Measurement
  Eigen::VectorXd z_ = Eigen::VectorXd::Zero(MEASUREMENT_DIM);

  // Measurement noise
  Eigen::MatrixXd R_ = Eigen::MatrixXd::Zero(MEASUREMENT_DIM, MEASUREMENT_DIM);

  rclcpp::Subscription<orca_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Publisher<orca_msgs::msg::Barometer>::SharedPtr filtered_baro_pub_;

  // Callback wrapper, guarantees timestamp monotonicity
  monotonic::Monotonic<BaroFilterNode *, const orca_msgs::msg::Barometer::SharedPtr>
    baro_cb_{this, &BaroFilterNode::baro_callback};

  // Barometer callback
  void baro_callback(orca_msgs::msg::Barometer::SharedPtr msg, bool first)
  {
    orca_msgs::msg::Barometer filtered_baro_msg = *msg;

    if (first) {
      // Initialize the state
      auto x = filter_.x();
      x(0) = msg->pressure;
      filter_.set_x(x);
      RCLCPP_INFO(get_logger(), "baro_filter_node ready");
    } else {
      // Run the filter
      filter_.predict(baro_cb_.dt(), u_);
      z_(0) = msg->pressure;
      R_(0, 0) = msg->pressure_variance;
      filter_.update(z_, R_);
      const auto & x = filter_.x();
      // std::cout << "p: " << x(0) << ", vp: " << x(1) << ", ap: " << x(2) << std::endl;
      filtered_baro_msg.pressure = x(0);
    }

    filtered_baro_pub_->publish(filtered_baro_msg);
  }

public:
  BaroFilterNode()
    : Node{"baro_filter_node"}
  {
    (void) baro_sub_;

    baro_sub_ = create_subscription<orca_msgs::msg::Barometer>(
      "barometer",
      QUEUE_SIZE,
      [this](const orca_msgs::msg::Barometer::SharedPtr msg) -> void
        { this->baro_cb_.call(msg); });

    // Advertise
    filtered_baro_pub_ = create_publisher<orca_msgs::msg::Barometer>(
      "filtered_barometer",
      QUEUE_SIZE);

    // State transition function
    filter_.set_f_fn([](const double dt, const Eigen::VectorXd & u, Eigen::Ref<Eigen::VectorXd> x)
      {
        // Ignore u
        // ap is discovered

        // vp += ap * dt
        x(1) += x(2) * dt;

        // p += vp * dt
        x(0) += x(1) * dt;
      });

    // Measurement function
    filter_.set_h_fn([](const Eigen::Ref<const Eigen::VectorXd> & x, Eigen::Ref<Eigen::VectorXd> z)
      {
        z(0) = x(0);
      });

    // Process noise
    filter_.set_Q(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * PROCESS_NOISE);
  }

  ~BaroFilterNode() override = default;
};

}  // namespace orca_filter

//=============================================================================
// Main
//=============================================================================

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Init node
  auto node = std::make_shared<orca_filter::BaroFilterNode>();

  // Spin node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
