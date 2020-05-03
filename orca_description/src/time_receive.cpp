#include <cmath>
#include <iomanip>
#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"

constexpr int QUEUE_SIZE = 10;
constexpr int NUM_MEASUREMENTS = 300;

double mean(const std::vector<double> &v)
{
  double sum = 0;
  for (const auto &m : v) {
    sum += m;
  }
  return sum / v.size();
}

double stdev(const std::vector<double> &v, double u)
{
  double squared = 0;
  for (const auto &m : v) {
    squared += ((m - u) * (m - u));
  }
  return sqrt(squared / v.size());
}

class TimeReceiveNode : public rclcpp::Node
{
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_;
  bool receiving_;
  std::vector<double> values_;

public:

  TimeReceiveNode() : Node{"time_receive_node"}
  {
    (void) sub_;

    values_.reserve(NUM_MEASUREMENTS);

    sub_ = this->create_subscription<std_msgs::msg::Header>(
      "time_test", QUEUE_SIZE,
      [this](std_msgs::msg::Header::UniquePtr msg)
      {
        if (!this->receiving_) {
          this->receiving_ = true;
          std::cout << "receiving messages" << std::endl;
        }

        values_.push_back((this->now() - msg->stamp).seconds() * 1e6);
        if (values_.size() >= NUM_MEASUREMENTS) {
          auto u = mean(this->values_);
          auto s = stdev(this->values_, u);
          std::cout << std::fixed << std::setprecision(0)
                    << "average lag over " << NUM_MEASUREMENTS << " measurements: "
                    << u << " μs +/- " << s << std::endl;
          this->values_.clear();
          this->values_.reserve(NUM_MEASUREMENTS);
        }
      });
  }
};

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeReceiveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
