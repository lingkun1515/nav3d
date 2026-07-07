#pragma once

#include "global_reloc/feature.hpp"
#include "global_reloc/types.hpp"
#include "global_reloc/point_types.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <memory>

namespace global_reloc {

/// A pre-built global map with features for coarse relocalization.
/// Built offline, serialized to disk, loaded online.
struct GlobalMap {
  CloudRNPtr cloud;                                  ///< downsampled xyz+normal
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh;   ///< parallel FPFH
  std::vector<float> intensity;                      ///< per-point reflectance (parallel to cloud); empty if N/A
  AABB aabb;                                         ///< map bounding box
  double voxel_size = 0.0;                           ///< the voxel used at build time

  size_t size() const { return cloud ? cloud->size() : 0; }
  bool empty() const { return cloud ? cloud->empty() : true; }
  bool hasIntensity() const { return !intensity.empty(); }
};

/// Serialize a GlobalMap to a directory:
///   <dir>/<name>.pcd       (xyz+normal, PCL PCD binary)
///   <dir>/<name>.fpfh.bin  (raw FPFH 33*f32 per point + header)
/// Returns the path to the .gkey index file written.
std::string saveGlobalMap(const GlobalMap& m, const std::string& dir, const std::string& name);

/// Load a GlobalMap previously saved. path is the .gkey index file.
GlobalMap loadGlobalMap(const std::string& gkey_path);

/// Build a GlobalMap from a raw PCD (any XYZ-compatible point type on disk).
/// Pipeline: load -> voxel downsample -> normals -> FPFH -> AABB.
class MapBuilder {
 public:
  struct BuildOptions {
    double voxel_size = 0.5;          ///< map downsample voxel [m]
    FeatureParams feature;            ///< normal/fpfh radii
    bool keep_intensity = true;       ///< false: drop reflectance (geometry-only map)
  };

  explicit MapBuilder(const BuildOptions& opts) : opts_(opts) {}

  /// Build from a PCD file on disk.
  GlobalMap buildFromPCD(const std::string& pcd_path) const;

  /// Convenience: build + save to <dir>/<name>.
  std::string buildAndSave(const std::string& pcd_path,
                           const std::string& out_dir,
                           const std::string& name) const;

 private:
  BuildOptions opts_;
};

}  // namespace global_reloc
