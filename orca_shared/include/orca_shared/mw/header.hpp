#ifndef ORCA_SHARED_MW_HEADER_HPP
#define ORCA_SHARED_MW_HEADER_HPP

#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace mw
{

  class Header
  {
    std_msgs::msg::Header msg_;

  public:

    Header() = default;

    explicit Header(const std_msgs::msg::Header &msg) :
      msg_{msg}
    {}

    Header(const rclcpp::Time &stamp, const std::string &frame_id)
    {
      msg_.stamp = stamp;
      msg_.frame_id = frame_id;
    }

    std_msgs::msg::Header msg() const
    {
      return msg_;
    }

    rclcpp::Time stamp() const
    {
      return rclcpp::Time{msg_.stamp, RCL_ROS_TIME};
    }

    std::string frame_id() const
    {
      return msg_.frame_id;
    }

    void stamp(const rclcpp::Time &v)
    {
      msg_.stamp = v;
    }

    void frame_id(const std::string &v)
    {
      msg_.frame_id = v;
    }

    bool valid_stamp()
    {
      return stamp().nanoseconds() > 0;
    }

    bool operator==(const Header &that) const
    {
      return msg_ == that.msg_;
    }

    bool operator!=(const Header &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Header &v);
  };

}

#endif //ORCA_SHARED_MW_HEADER_HPP
