#pragma once

#include "global_reloc/types.hpp"
#include "global_reloc/map_builder.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include <memory>
#include <vector>

namespace global_reloc {

/// NDT-based coarse matcher with GICP refinement.
///
/// Uses PCL's NormalDistributionsTransform which has a MUCH wider convergence
/// basin (~5-10m) than GICP (~1-2m). This makes 5m grid search feasible:
/// ~200 NDT runs (fast, ~0.1s each) → top-5 → GICP refinement (precise).
///
/// Pipeline:
/// 1. 5m grid × 4 yaw = ~200 NDT inits → each runs NDT (fast, wide basin)
/// 2. Top-5 NDT results → GICP refinement (narrow basin, precise)
/// 3. Pick best by tight_inlier_ratio
class NdtGicpMatcher {
 public:
  explicit NdtGicpMatcher(const BevParams& params);
  ~NdtGicpMatcher();

  void setMap(const GlobalMap* map);
  std::vector<Candidate> match(const pcl::PointCloud<pcl::PointXYZ>& query_raw) const;

  struct NdtResult { Eigen::Isometry3d pose; double fitness; };
  /// Ensure the NDT + small_gicp structures are built (lazy-init). Safe to call
  /// repeatedly; no-op when already built.
  void ensureBuilt() const;

  /// Run a single NDT alignment from @p init (for external per-candidate refinement).
  NdtResult runNdt(const pcl::PointCloud<pcl::PointXYZ>& query,
                   const Eigen::Isometry3d& init) const;

 private:
  BevParams params_;
  const GlobalMap* map_ = nullptr;

  // PCL NDT (preconfigured, reused)
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_xyz_;
  pcl::PointCloud<pcl::PointNormal>::Ptr map_normals_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_tree_pcl_;

  // small_gicp (for GICP refinement)
  small_gicp::PointCloud::Ptr map_pc_;
  small_gicp::KdTree<small_gicp::PointCloud>::Ptr map_tree_sg_;

  void build();

  struct GicpResult { Eigen::Isometry3d pose; double tight; double inlier; double resid; double precision; double normal_consistency; };
  GicpResult runGicp(const std::vector<Eigen::Vector3d>& query_pts,
                     const Eigen::Isometry3d& init) const;
};

}  // namespace global_reloc
