#ifndef ORCA_BASE_MESSAGE_QUEUE_H
#define ORCA_BASE_MESSAGE_QUEUE_H

#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

namespace orca_base
{

  template<typename T>
  class MessageQueue
  {
    rclcpp::Logger logger_;

    std::list<T> queue_{};

  public:

    explicit MessageQueue(const rclcpp::Logger &logger) : logger_{logger}
    {}

    void push(const T &msg)
    {
      queue_.push_back(msg);
    }

    bool msg_ready(const rclcpp::Time &start, const rclcpp::Time &end, rclcpp::Time &msg_time)
    {
      // Sort by time
      queue_.sort([](const T &a, const T &b) -> bool
                  {
                    rclcpp::Time a_stamp{a.header.stamp};
                    rclcpp::Time b_stamp{b.header.stamp};
                    return a_stamp < b_stamp;
                  });

      // Toss any messages older than start
      while (!queue_.empty() && rclcpp::Time{queue_.front().header.stamp} < start) {
        RCLCPP_WARN(logger_, "message of type %s %s is older than %s, dropping", typeid(T).name(),
                    to_str(rclcpp::Time{queue_.front().header.stamp}).c_str(), to_str(start).c_str());
        queue_.pop_front();
      }

      // If there's a message between start and end, return true
      if (queue_.empty() || rclcpp::Time{queue_.front().header.stamp} > end) {
        return false;
      } else {
        msg_time = rclcpp::Time{queue_.front().header.stamp};
        return true;
      }
    }

    T pop()
    {
      assert(!queue_.empty());
      T msg = queue_.front();
      queue_.pop_front();
      return msg;
    }
  };

} // namespace orca_base

#endif // ORCA_BASE_MESSAGE_QUEUE_H
