#pragma once

#include "global_reloc/feature.hpp"
#include "global_reloc/types.hpp"
#include "global_reloc/map_builder.hpp"

#include <vector>
#include <memory>

namespace global_reloc {

/// Coarse matcher: FPFH correspondence + SAC-IA, returns Top-K hypotheses.
/// Stateless w.r.t. the map — bind a map once, query many times.
class CoarseMatcher {
 public:
  explicit CoarseMatcher(const CoarseParams& params);

  /// Bind the pre-built global map (must already have FPFH).
  void setMap(const GlobalMap* map) { map_ = map; }

  /// Run coarse matching. Returns up to top_k Candidate objects (pose set,
  /// not refined; inlier_ratio/score filled as SAC fitness).
  std::vector<Candidate> match(const FeatureCloud& query) const;

 private:
  CoarseParams params_;
  const GlobalMap* map_ = nullptr;
};

}  // namespace global_reloc
