#include "clearance_policy.hpp"
#include <limits>
#include <stdexcept>

void require(bool condition)
{
  if (!condition) throw std::runtime_error("Clearance recovery policy regression");
}

int main()
{
  using P = ClearancePolicy;
  P clear({.1, .1}, {.005, .003});
  require(clear.accepts({.01, .01}) && clear.finishes(true));
  require(!clear.accepts({.004, .01}));

  P retreat({.1, .002976}, {.005, .003});
  require(retreat.recovering());
  require(retreat.accepts({.1, .00298}));
  require(retreat.accepts({.1, .00299}) && retreat.finishes(true));
  require(retreat.accepts({.1, .0031}));
  require(!retreat.accepts({.1, .00299}));  // No re-entry after clearing margin.

  P deeper({.1, .002976}, {.005, .003});
  require(!deeper.accepts({.1, .002975}));
  P detour({.1, .002976}, {.005, .003});
  require(!detour.accepts({.1, .0029}));  // Clear endpoint cannot excuse unsafe intermediate motion.

  P holding({.1, .002976}, {.005, .003});
  require(holding.accepts({.1, .002976}) && holding.finishes(false));
  require(!holding.finishes(true));  // No lateral motion while inside margin.

  P both({.004, .002}, {.005, .003});
  require(both.accepts({.0041, .002}));
  require(both.finishes(true));
  require(!both.accepts({.0042, .0019}));  // Retreat from floor cannot reduce self clearance.
  P other({.1, .002}, {.005, .003});
  require(!other.accepts({.004, .0021}));  // Nor may retreat enter a different safety margin.

  P numerical({.1, .002976}, {.005, .003});
  for (int i = 0; i < 5; ++i) require(numerical.accepts({.1, .002976 - P::tolerance / 2}));
  require(!numerical.accepts({.1, .002976 - P::tolerance * 2}));
  require(!P::measurable({.1, 0.}));
  require(!P::measurable({.1, -.001}));
  require(!P::measurable({.1, std::numeric_limits<double>::quiet_NaN()}));
  return 0;
}
