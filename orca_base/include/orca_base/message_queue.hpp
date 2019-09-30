#ifndef ORCA_BASE_MESSAGE_QUEUE_H
#define ORCA_BASE_MESSAGE_QUEUE_H

#include <queue>

#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

namespace orca_base
{

  template<typename T>
  class MessageQueue
  {
    rclcpp::Logger logger_;

    std::queue<T> queue_{};

    // Pop messages older than t
    void pop_older_than(const rclcpp::Time &t)
    {
      while (!queue_.empty() && rclcpp::Time{queue_.front().header.stamp} < t) {
        RCLCPP_WARN(logger_, "message of type %s was in the queue too long, dropping", typeid(T).name());
        queue_.pop();
      }
    }

  public:

    explicit MessageQueue(const rclcpp::Logger &logger) : logger_{logger}
    {}

    void push(const T &msg)
    {
      queue_.push(msg);
    }

    // Pop any messages older than start, and peek at the 1st message between start and end
    bool msg_ready(const rclcpp::Time &start, const rclcpp::Time &end, rclcpp::Time &msg_time)
    {
      pop_older_than(start);

      if (queue_.empty() || rclcpp::Time{queue_.front().header.stamp} > end) {
        return false;
      } else {
        msg_time = rclcpp::Time{queue_.front().header.stamp};
        return true;
      }
    }

    // Pop a message
    T pop_msg()
    {
      assert(!queue_.empty());
      T msg = queue_.front();
      queue_.pop();
      return msg;
    }
  };

} // namespace orca_base

#endif // ORCA_BASE_MESSAGE_QUEUE_H
