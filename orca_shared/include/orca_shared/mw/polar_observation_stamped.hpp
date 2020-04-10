#ifndef ORCA_SHARED_MW_POLAR_OBSERVATION_STAMPED_HPP
#define ORCA_SHARED_MW_POLAR_OBSERVATION_STAMPED_HPP

#include "orca_shared/mw/header.hpp"
#include "orca_shared/mw/polar_observation.hpp"

namespace mw
{

  class PolarObservationStamped
  {
    Header header_;
    PolarObservation observation_;

  public:

    PolarObservationStamped() = default;

    PolarObservationStamped(const Header &header, const PolarObservation &observation) :
      header_{header},
      observation_{observation}
    {}

    const Header &header() const
    {
      return header_;
    }

    const PolarObservation &observation() const
    {
      return observation_;
    }

    Header &header()
    {
      return header_;
    }

    PolarObservation &observation()
    {
      return observation_;
    }

    bool operator==(const PolarObservationStamped &that) const
    {
      return header_ == that.header_ &&
             observation_ == that.observation_;
    }

    bool operator!=(const PolarObservationStamped &that) const
    {
      return !(*this == that);
    }

    friend std::ostream &operator<<(std::ostream &os, PolarObservationStamped const &v);
  };

}

#endif //ORCA_SHARED_MW_POLAR_OBSERVATION_STAMPED_HPP
