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
#include <chrono>

namespace global_reloc {

/// Fast global matcher: NDT coarse position + chamfer yaw sweep + GICP refine.
///
/// Key insight: the bottleneck is yaw ambiguity in structured environments.
/// GICP from wrong yaw converges to wrong local optimum. Instead:
/// 1. NDT finds rough position (wide basin, ~10m grid, fast)
/// 2. At each NDT position, do a fast chamfer yaw sweep (bidirectional NN,
///    no iteration) to find the best yaw — chamfer discriminates yaw better
///    than GICP because it's bidirectional and non-iterative (no local optima)
/// 3. GICP refines from (position, best_yaw)
///
/// Target: <1s per reloc on a 67×62m map.
class FastGlobalMatcher {
 public:
  explicit FastGlobalMatcher(const BevParams& params);
  ~FastGlobalMatcher();

  void setMap(const GlobalMap* map);

  std::vector<Candidate> match(const pcl::PointCloud<pcl::PointXYZ>& query_raw) const;

  /// Last run timing (ms): [ndt_ms, chamfer_ms, gicp_ms, total_ms]
  mutable std::array<double, 4> last_timing_ms{};

 private:
  BevParams params_;
  const GlobalMap* map_ = nullptr;

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_xyz_;
  pcl::PointCloud<pcl::PointNormal>::Ptr map_normals_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_tree_;

  // Precomputed map radial profiles for fast yaw estimation.
  struct RadialProfile { double x, y; std::vector<double> rmax; };
  std::vector<RadialProfile> map_radial_;

  small_gicp::PointCloud::Ptr map_pc_;
  small_gicp::KdTree<small_gicp::PointCloud>::Ptr map_tree_sg_;

  void ensureBuilt() const;
  void build();

  /// Bidirectional chamfer score at a given pose.
  /// recall = fraction of query points with map NN < thresh
  /// precision = fraction of map points (in query footprint) with query NN < thresh
  /// returns recall * precision (higher = better)
  double chamferScore(const pcl::PointCloud<pcl::PointXYZ>& query_aligned,
                      double thresh) const;

  /// GICP refine from init pose.
  struct GicpResult { Eigen::Isometry3d pose; double tight; double inlier; double resid; };
  GicpResult runGicp(const std::vector<Eigen::Vector3d>& query_pts,
                     const Eigen::Isometry3d& init) const;
};

}  // namespace global_reloc
