#pragma once

#include "global_reloc/point_types.hpp"
#include "global_reloc/types.hpp"

#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>

#include <memory>

namespace global_reloc {

using CloudRN = pcl::PointCloud<PointXYZRN>;
using CloudRNPtr = CloudRN::Ptr;

/// Result of feature extraction: downsampled cloud with normals + parallel FPFH.
struct FeatureCloud {
  CloudRNPtr cloud;                                  ///< downsampled xyz+normal
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh;   ///< parallel FPFH (same indexing)
};

/// Extract features (downsample -> normals -> FPFH) from a raw XYZ cloud.
/// `num_threads` <= 0 means auto (omp max threads).
/// Throws std::runtime_error on failure.
class FeatureExtractor {
 public:
  explicit FeatureExtractor(const FeatureParams& params);

  /// Voxel downsample + normal + FPFH. cloud_in may be any XYZ point type;
  /// caller should pass a pcl::PointCloud<pcl::PointXYZ> equivalent.
  FeatureCloud extract(const pcl::PointCloud<pcl::PointXYZ>& raw,
                       double voxel_size_override = 0.0) const;

  /// Compute only normals (skip FPFH). Used for sanity/gravity checks.
  CloudRNPtr computeNormals(const pcl::PointCloud<pcl::PointXYZ>& raw,
                            double voxel_size) const;

  const FeatureParams& params() const { return params_; }

 private:
  FeatureParams params_;
  int resolvedThreads() const;
};

}  // namespace global_reloc
