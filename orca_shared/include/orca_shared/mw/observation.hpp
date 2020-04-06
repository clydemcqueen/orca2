#ifndef ORCA_SHARED_MW_OBSERVATION_HPP
#define ORCA_SHARED_MW_OBSERVATION_HPP

#include "fiducial_vlam_msgs/msg/observation.hpp"
#include "opencv2/core/types.hpp"

namespace mw
{

  constexpr int NOT_A_MARKER = -1;

  class Observation
  {
    fiducial_vlam_msgs::msg::Observation msg_;

  public:

    static const Observation None;

    Observation() = default;

    explicit Observation(const fiducial_vlam_msgs::msg::Observation &msg) :
      msg_{msg}
    {}

    Observation(const int &id,
                const cv::Point2d &c0, const cv::Point2d &c1, const cv::Point2d &c2, const cv::Point2d &c3)
    {
      msg_.id = id;
      msg_.x0 = c0.x;
      msg_.y0 = c0.y;
      msg_.x1 = c1.x;
      msg_.y1 = c1.y;
      msg_.x2 = c2.x;
      msg_.y2 = c2.y;
      msg_.x3 = c3.x;
      msg_.y3 = c3.y;
    }

    fiducial_vlam_msgs::msg::Observation msg() const
    {
      return msg_;
    }

    int id() const
    {
      return msg_.id;
    }

    cv::Point2d c0() const
    {
      return cv::Point2d{msg_.x0, msg_.y0};
    }

    cv::Point2d c1() const
    {
      return cv::Point2d{msg_.x1, msg_.y1};
    }

    cv::Point2d c2() const
    {
      return cv::Point2d{msg_.x2, msg_.y2};
    }

    cv::Point2d c3() const
    {
      return cv::Point2d{msg_.x3, msg_.y3};
    }

    bool operator==(const Observation &that) const
    {
      return msg_ == that.msg_;
    }

    bool operator!=(const Observation &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Observation &v);
  };

}

#endif //ORCA_SHARED_MW_OBSERVATION_HPP
