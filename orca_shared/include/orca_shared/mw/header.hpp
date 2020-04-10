#ifndef ORCA_SHARED_MW_HEADER_HPP
#define ORCA_SHARED_MW_HEADER_HPP

#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace mw
{

  class Header
  {
    rclcpp::Time t_;
    std::string frame_id_;

  public:

    Header() = default;

    explicit Header(const std_msgs::msg::Header &msg) :
      t_{msg.stamp, RCL_ROS_TIME},
      frame_id_{msg.frame_id}
    {}

    Header(const rclcpp::Time &stamp, const std::string &frame_id)
    {
      t_ = stamp;
      frame_id_ = frame_id;
    }

    std_msgs::msg::Header msg() const
    {
      std_msgs::msg::Header msg;
      msg.stamp = t_;
      msg.frame_id = frame_id_;
      return msg;
    }

    bool valid()
    {
      return t().nanoseconds() > 0;
    }

    const rclcpp::Time &t() const
    {
      return t_;
    }

    const std::string &frame_id() const
    {
      return frame_id_;
    }

    rclcpp::Time &t()
    {
      return t_;
    }

    std::string &frame_id()
    {
      return frame_id_;
    }

    bool operator==(const Header &that) const
    {
      return t_ == that.t_ &&
             frame_id_ == that.frame_id_;
    }

    bool operator!=(const Header &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Header &v);
  };

}

#endif //ORCA_SHARED_MW_HEADER_HPP
