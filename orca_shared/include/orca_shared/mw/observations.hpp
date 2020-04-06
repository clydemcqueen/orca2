#ifndef ORCA_SHARED_MW_OBSERVATIONS_HPP
#define ORCA_SHARED_MW_OBSERVATIONS_HPP

#include "orca_msgs/msg/observations2.hpp"
//#include "orca_shared/mw/map.hpp"
#include "orca_shared/mw/observer.hpp"
#include "orca_shared/mw/observation.hpp"
#include "orca_shared/mw/polar_observation.hpp"
#include "orca_shared/mw/pose.hpp"

namespace mw
{

  class Observations
  {
    Observer observer_;
    std::vector<Observation> observations_;
    std::vector<PolarObservation> polar_observations_;

  public:

    Observations() = default;

    explicit Observations(const orca_msgs::msg::Observations2 &msg) :
      observer_{msg.observer}
    {
      assert(msg.observations.size() == msg.polar_observations.size());
      for (const auto &item : msg.observations) {
        observations_.emplace_back(item);
      }
      for (const auto &item : msg.polar_observations) {
        polar_observations_.emplace_back(item);
      }
    }

    Observations(const Observer &observer) :
      observer_{observer}
    {}

    orca_msgs::msg::Observations2 msg() const
    {
      orca_msgs::msg::Observations2 msg;
      msg.observer = observer_.msg();
      for (const auto &item : observations_) {
        msg.observations.emplace_back(item.msg());
      }
      for (const auto &item : polar_observations_) {
        msg.polar_observations.emplace_back(item.msg());
      }
      return msg;
    }

    const Observer &observer() const
    {
      return observer_;
    }

    Observer &observer()
    {
      return observer_;
    }

    bool empty() const
    {
      return observations_.empty();
    }

    void clear()
    {
      observations_.clear();
      polar_observations_.clear();
    }

    const Observation &get(const int &id) const
    {
      for (const auto &item : observations_) {
        if (id == item.id()) {
          return item;
        }
      }
      return Observation::None;
    }

    const PolarObservation &get_polar(const int &id) const
    {
      for (const auto &item : polar_observations_) {
        if (id == item.id()) {
          return item;
        }
      }
      return PolarObservation::None;
    }

    void add(const Observation &observation)
    {
      observations_.push_back(observation);
      polar_observations_.push_back(observer_.polar_observation(observation));
    }

    void add_polar(const PolarObservation &polar_observation)
    {
      observations_.push_back(observer_.observation(polar_observation));
      polar_observations_.push_back(polar_observation);
    }

    bool operator==(const Observations &that) const
    {
      if (observations_.size() != that.observations_.size()) {
        return false;
      }
      for (int i = 0; i < observations_.size(); ++i) {
        if (observations_[i] != that.observations_[i]) {
          return false;
        }
      }

      return true;
    }

    bool operator!=(const Observations &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, const Observations &v);
  };

}

#endif //ORCA_SHARED_MW_OBSERVATIONS_HPP
