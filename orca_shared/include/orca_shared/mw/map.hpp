#ifndef ORCA_SHARED_MW_MAP_HPP
#define ORCA_SHARED_MW_MAP_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "orca_shared/mw/marker.hpp"
#include "orca_shared/mw/observations.hpp"
#include "orca_shared/mw/pose.hpp"

namespace mw
{

  class Map
  {
    fiducial_vlam_msgs::msg::Map msg_;
    std::vector<mw::Marker> markers_;

  public:

    Map() = default;

    explicit Map(const fiducial_vlam_msgs::msg::Map &msg) :
      msg_{msg}
    {
      for (int i = 0; i < msg_.ids.size(); ++i) {
        markers_.emplace_back(msg_.ids[i], msg_.marker_length, msg_.poses[i].pose);
      }
    }

    fiducial_vlam_msgs::msg::Map msg() const
    {
      return msg_;
    }

    int predict_observations(const Pose &base_f_map, Observations &observations);

    bool operator==(const Map &that) const
    {
      return msg_ == that.msg_;
    }

    bool operator!=(const Map &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Map &v);
  };

}

#endif //ORCA_SHARED_MW_MAP_HPP
