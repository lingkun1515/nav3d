#pragma once

#include "global_reloc/types.hpp"
#include "global_reloc/map_builder.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include <memory>
#include <vector>

namespace global_reloc {

/// NDT/GICP-based coarse matcher.
///
/// Replaces the BEV occupancy approach which could not disambiguate yaw in
/// repetitive indoor structure. This matcher does what super_lio's reloc does:
/// for each candidate yaw (coarse grid), initialize a 3D GICP registration of
/// the query against the full 3D map and let GICP converge. The 3D structure
/// (height, wall layout) constrains the solution far better than a 2D BEV.
///
/// Only yaw is searched globally (robot localization: z/roll/pitch are
/// constrained). The search is coarse-to-fine: 30° steps → refine top-K ±15°
/// at 5° steps, each with a fast GICP convergence.
class NdtMatcher {
 public:
  explicit NdtMatcher(const BevParams& params);
  ~NdtMatcher();

  void setMap(const GlobalMap* map);

  std::vector<Candidate> match(const pcl::PointCloud<pcl::PointXYZ>& query_raw) const;

  /// Variant that uses BEV candidates as GICP init positions (much faster
  /// than the dense grid: ~80 runs vs ~3700).
  std::vector<Candidate> matchFromCandidates(
      const pcl::PointCloud<pcl::PointXYZ>& query_raw,
      const std::vector<Candidate>& bev_candidates) const;

 private:
  BevParams params_;
  const GlobalMap* map_ = nullptr;

  small_gicp::PointCloud::Ptr map_pc_;
  small_gicp::KdTree<small_gicp::PointCloud>::Ptr map_tree_;
  // Also keep a PCL KdTree for overlap evaluation (shared with old code path).
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_xyz_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_tree_pcl_;

  void ensureBuilt() const;
  void build();

  /// Run GICP from a given init pose, return refined pose + fitness.
  struct GicpResult { Eigen::Isometry3d pose; double inlier_ratio; double tight_ratio; double resid; double E; bool converged; };
  GicpResult runGicp(const std::vector<Eigen::Vector3d>& query_pts,
                     const Eigen::Isometry3d& init_T) const;
};

}  // namespace global_reloc
