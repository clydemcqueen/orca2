#ifndef ORCA_SHARED_MW_MAP_HPP
#define ORCA_SHARED_MW_MAP_HPP

#include "fiducial_vlam_msgs/msg/map.hpp"
#include "orca_shared/mw/header.hpp"
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
        markers_.emplace_back(msg_.ids[i], msg_.marker_length, Pose{msg_.poses[i].pose});
      }
    }

    fiducial_vlam_msgs::msg::Map msg() const
    {
      return msg_;
    }

    bool valid() const
    {
      return Header{msg_.header}.valid();
    }

    double marker_length() const
    {
      return msg_.marker_length;
    }

    const std::vector<mw::Marker> &markers() const
    {
      return markers_;
    }

    const Marker &get(const int &id) const
    {
      for (const auto &item : markers_) {
        if (id == item.id()) {
          return item;
        }
      }
      return Marker::None;
    }

    /**
     * Plan a route from start to destination through the smallest number of waypoints.
     *
     * Ignores marker orientation, so only works for down-facing cameras and markers on the seafloor.
     *
     * @param target_z Travel depth
     * @param max_dead_reckon_dist Max dead reckoning distance
     * @param start_pose Start pose
     * @param destination_pose Destination pose
     * @param waypoints Out: all posts from start to destionat
     * @return True if a path was found
     */
    bool get_waypoints(const double &target_z, const double &max_dead_reckon_dist,
                       const Pose &start_pose, const Pose &destination_pose, std::vector<Pose> &waypoints) const;

    /**
     * @deprecated
     * @param base_f_map
     * @param observations
     * @return
     */
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
