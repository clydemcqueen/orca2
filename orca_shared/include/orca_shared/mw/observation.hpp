#ifndef ORCA_SHARED_MW_OBSERVATION_HPP
#define ORCA_SHARED_MW_OBSERVATION_HPP

#include "fiducial_vlam_msgs/msg/observation.hpp"
#include "opencv2/core/types.hpp"

namespace mw
{

  constexpr int NOT_A_MARKER = -1;

  class Observation
  {
    int id_;
    cv::Point2d c0_;
    cv::Point2d c1_;
    cv::Point2d c2_;
    cv::Point2d c3_;

  public:

    static const Observation None;

    Observation() = default;

    explicit Observation(const fiducial_vlam_msgs::msg::Observation &msg)
    {
      id_ = msg.id;
      c0_ = cv::Point2d{msg.x0, msg.y0};
      c1_ = cv::Point2d{msg.x1, msg.y1};
      c2_ = cv::Point2d{msg.x2, msg.y2};
      c3_ = cv::Point2d{msg.x3, msg.y3};
    }

    Observation(const int &id,
                const cv::Point2d &c0, const cv::Point2d &c1, const cv::Point2d &c2, const cv::Point2d &c3) :
      id_{id},
      c0_{c0},
      c1_{c1},
      c2_{c2},
      c3_{c3}
    {}

    fiducial_vlam_msgs::msg::Observation msg() const
    {
      fiducial_vlam_msgs::msg::Observation msg;
      msg.id = id_;
      msg.x0 = c0_.x;
      msg.y0 = c0_.y;
      msg.x1 = c1_.x;
      msg.y1 = c1_.y;
      msg.x2 = c2_.x;
      msg.y2 = c2_.y;
      msg.x3 = c3_.x;
      msg.y3 = c3_.y;
      return msg;
    }

    const int &id() const
    {
      return id_;
    }

    const cv::Point2d &c0() const
    {
      return c0_;
    }

    const cv::Point2d &c1() const
    {
      return c1_;
    }

    const cv::Point2d &c2() const
    {
      return c2_;
    }

    const cv::Point2d &c3() const
    {
      return c3_;
    }

    int id()
    {
      return id_;
    }

    const cv::Point2d &c0()
    {
      return c0_;
    }

    const cv::Point2d &c1()
    {
      return c1_;
    }

    const cv::Point2d &c2()
    {
      return c2_;
    }

    const cv::Point2d &c3()
    {
      return c3_;
    }

    bool operator==(const Observation &that) const
    {
      return id_ == that.id_ &&
             c0_ == that.c0_ &&
             c1_ == that.c1_ &&
             c2_ == that.c2_ &&
             c3_ == that.c3_;
    }

    bool operator!=(const Observation &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Observation &v);
  };

}

#endif //ORCA_SHARED_MW_OBSERVATION_HPP
