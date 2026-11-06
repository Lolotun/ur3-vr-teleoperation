#pragma once
#include <algorithm>
#include <array>
#include <cmath>

// Permit retreat from a positive clearance below its configured margin. This
// never authorizes contact: geometric collision detection runs before policy.
class ClearancePolicy
{
public:
  using Distances = std::array<double, 2>;  // scene, self
  static constexpr double tolerance = 1e-7;  // numerical tolerance, meters

  ClearancePolicy(Distances initial, Distances margins)
    : initial_(initial), previous_(initial), margins_(margins)
  {
    for (std::size_t i = 0; i < initial.size(); ++i)
      recovering_[i] = initial[i] + tolerance < margins[i];
  }

  static bool measurable(const Distances& distances)
  {
    return std::all_of(distances.begin(), distances.end(),
                       [](double value) { return std::isfinite(value) && value > 0.; });
  }

  bool accepts(const Distances& sample)
  {
    if (!measurable(sample)) return false;
    for (std::size_t i = 0; i < sample.size(); ++i)
    {
      // Before restoring the margin, require a nondecreasing minimum gap.
      // After restoring it, forbid re-entry into the proximity band. Anchor
      // at initial_ as well so numerical tolerance cannot accumulate drift.
      const auto lower = recovering_[i] ? std::min(margins_[i], std::max(initial_[i], previous_[i])) : margins_[i];
      if (sample[i] + tolerance < lower) return false;
    }
    previous_ = sample;
    return true;
  }

  bool recovering() const { return recovering_[0] || recovering_[1]; }

  bool finishes(bool moving) const
  {
    if (!moving || !recovering()) return true;  // Holding measured Q is allowed.
    for (std::size_t i = 0; i < initial_.size(); ++i)
      if (recovering_[i] && previous_[i] > initial_[i] + tolerance) return true;
    return false;  // A moving command must actually improve a restricted gap.
  }

private:
  Distances initial_, previous_, margins_;
  std::array<bool, 2> recovering_{};
};
