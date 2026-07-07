#pragma once

#include "global_reloc/feature.hpp"
#include "global_reloc/types.hpp"
#include "global_reloc/map_builder.hpp"
#include "global_reloc/point_types.hpp"

#include <vector>

namespace global_reloc {

/// Refine coarse hypotheses with GICP. small_gicp (header-only) is the
/// primary backend when present; falls back to PCL GICP otherwise.
class FineMatcher {
 public:
  explicit FineMatcher(const FineParams& params);

  /// Bind the pre-built map. Internally builds a KdTree/voxel structure lazily.
  void setMap(const GlobalMap* map);

  /// Refine a list of candidates in place. Sets `refined=true`, updates
  /// pose, inlier_ratio, tight_inlier_ratio, intensity_consistency,
  /// mean_residual, num_inliers, overlap. If query_intensity is provided (and
  /// the map carries reflectance), intensity_consistency is computed to
  /// disambiguate repetitive geometry.
  void refine(std::vector<Candidate>& candidates,
              const pcl::PointCloud<pcl::PointXYZ>& query_raw,
              const std::vector<float>* query_intensity = nullptr) const;

 private:
  FineParams params_;
  const GlobalMap* map_ = nullptr;
};

}  // namespace global_reloc
