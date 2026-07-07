#pragma once

// ROS-free facade over the global_reloc library. GUI/CLI/tests use this
// without pulling in rclcpp. Keeps ownership of the GlobalMap and the
// Relocalizer; exposes a tiny surface: build/load map, run estimate,
// query visualization buffers.

#include "global_reloc/relocalizer.hpp"
#include "global_reloc/map_builder.hpp"
#include "global_reloc/params.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>

namespace global_reloc {

/// Result of a single relocalization call, with everything the GUI needs.
struct RelocOutcome {
  RelocResult result;
  std::vector<Eigen::Vector3f> map_points;        ///< map xyz (downsampled view)
  std::vector<Eigen::Vector3f> query_raw;        ///< query xyz (raw, world-unaligned)
  std::vector<Eigen::Vector3f> query_aligned;     ///< query xyz after estimated T
  bool has_map = false;
};

class RelocFacade {
 public:
  /// Load params from a YAML file (or use defaults if path empty).
  explicit RelocFacade(const std::string& params_path = "");

  // ---- map management ----
  /// Build a GlobalMap from a PCD file on disk, save alongside, keep loaded.
  /// out_dir/name form the .gkey path; pass empty out_dir to skip saving.
  bool buildMapFromPCD(const std::string& pcd_path,
                       const std::string& out_dir = "",
                       const std::string& name = "map");

  /// Load a previously saved map (.gkey index file).
  bool loadMap(const std::string& gkey_path);

  bool hasMap() const { return static_cast<bool>(map_); }
  size_t mapSize() const { return map_ ? map_->size() : 0; }
  const GlobalMap& map() const { return *map_; }

  // ---- estimate ----
  /// Run relocalization on a raw XYZ cloud. If params.accumulate_frames>1
  /// the facade still does single-shot (passes the whole cloud).
  RelocOutcome estimate(const pcl::PointCloud<pcl::PointXYZ>& query);

  /// Provide gravity direction in the query/LiDAR frame for sanity check.
  void setGravity(const Eigen::Vector3d& g) { gravity_ = g; }

  const RelocParams& params() const { return params_; }
  void setVoxelMap(double v) { build_voxel_ = v; }

 private:
  void ensureRelocalizer();

  RelocParams params_;
  std::shared_ptr<GlobalMap> map_;
  std::unique_ptr<Relocalizer> reloc_;
  Eigen::Vector3d gravity_{0, 0, 0};
  double build_voxel_ = 0.5;
};

// ---- synthetic data helpers (so the GUI can demo without a real dataset) ----

/// Generate a synthetic structured scene (room with pillars). Returns XYZ.
pcl::PointCloud<pcl::PointXYZ> makeSyntheticRoom(int density = 4000);

/// Crop a local "LiDAR scan" from a scene at a viewpoint, optionally apply a
/// known perturbation (returns the perturbed scan and the GT inverse pose).
struct SyntheticQuery {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Isometry3d gt_pose;   ///< T_map_query ground truth
};
SyntheticQuery makeSyntheticQuery(const pcl::PointCloud<pcl::PointXYZ>& scene,
                                   const Eigen::Vector3d& viewpoint,
                                   double radius,
                                   const Eigen::Isometry3d& perturbation);

}  // namespace global_reloc
