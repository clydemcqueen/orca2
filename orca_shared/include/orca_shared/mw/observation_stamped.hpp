#ifndef ORCA_SHARED_MW_OBSERVATION_STAMPED_HPP
#define ORCA_SHARED_MW_OBSERVATION_STAMPED_HPP

#include "orca_shared/mw/header.hpp"
#include "orca_shared/mw/observation.hpp"

namespace mw
{

  class ObservationStamped
  {
    Header header_;
    Observation observation_;

  public:

    ObservationStamped() = default;

    ObservationStamped(const Header &header, const Observation &observation) :
      header_{header},
      observation_{observation}
    {}

    const Header &header() const
    {
      return header_;
    }

    const Observation &observation() const
    {
      return observation_;
    }

    Header &header()
    {
      return header_;
    }

    Observation &observation()
    {
      return observation_;
    }

    bool operator==(const ObservationStamped &that) const
    {
      return header_ == that.header_ &&
             observation_ == that.observation_;
    }

    bool operator!=(const ObservationStamped &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, ObservationStamped const &v);
  };

}

#endif //ORCA_SHARED_MW_OBSERVATION_STAMPED_HPP
