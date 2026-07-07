#pragma once

#include "global_reloc/types.hpp"

#include <vector>
#include <optional>

namespace global_reloc {

/// Score refined candidates and pick the best, applying sanity checks.
class Scorer {
 public:
  explicit Scorer(const ScoringParams& params);

  /// Compute composite score for one candidate and store in c.score [0,1].
  void scoreOne(Candidate& c, const AABB& map_aabb) const;

  /// Pick the best candidate that passes sanity checks.
  /// Returns std::nullopt (with reason set on result) if none converge.
  /// gravity_attitude: optional IMU-derived roll/pitch (as rotation matrix
  /// columns of a gravity-aligned frame) to validate against the map's gravity
  /// direction (assumed +Z up). If empty, gravity check is skipped.
  std::optional<RelocResult> pick(
      std::vector<Candidate> candidates,
      const AABB& map_aabb,
      const Eigen::Vector3d& gravity_attitude = Eigen::Vector3d::Zero()) const;

 private:
  ScoringParams params_;
  bool gravityValid(const Eigen::Vector3d& g) const {
    return g.squaredNorm() > 1e-6;
  }
};

}  // namespace global_reloc
