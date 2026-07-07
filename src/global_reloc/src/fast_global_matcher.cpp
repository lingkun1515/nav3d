#include "global_reloc/fast_global_matcher.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <cmath>
#include <algorithm>
#include <array>
#include <set>
#include <map>

namespace global_reloc {

namespace {
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDown(
    const pcl::PointCloud<pcl::PointXYZ>& raw, double voxel) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  if (voxel <= 0) { *out = raw; return out; }
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(raw.makeShared());
  vg.setLeafSize(voxel, voxel, voxel);
  vg.filter(*out);
  return out;
}
}  // namespace

FastGlobalMatcher::FastGlobalMatcher(const BevParams& params) : params_(params) {}
FastGlobalMatcher::~FastGlobalMatcher() = default;

void FastGlobalMatcher::setMap(const GlobalMap* map) {
  if (!map) throw std::runtime_error("FastGlobalMatcher: null map");
  map_ = map;
  ndt_.reset();
}

void FastGlobalMatcher::ensureBuilt() const {
  if (ndt_ && map_pc_) return;
  const_cast<FastGlobalMatcher*>(this)->build();
}

void FastGlobalMatcher::build() {
  if (!map_ || map_->empty()) throw std::runtime_error("FastGlobalMatcher: empty map");

  map_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  map_xyz_->resize(map_->cloud->size());
  map_normals_.reset(new pcl::PointCloud<pcl::PointNormal>);
  map_normals_->resize(map_->cloud->size());
  for (size_t i = 0; i < map_->cloud->size(); ++i) {
    map_xyz_->at(i).x = map_->cloud->at(i).x;
    map_xyz_->at(i).y = map_->cloud->at(i).y;
    map_xyz_->at(i).z = map_->cloud->at(i).z;
    map_normals_->at(i).x = map_->cloud->at(i).x;
    map_normals_->at(i).y = map_->cloud->at(i).y;
    map_normals_->at(i).z = map_->cloud->at(i).z;
    map_normals_->at(i).normal_x = map_->cloud->at(i).normal_x;
    map_normals_->at(i).normal_y = map_->cloud->at(i).normal_y;
    map_normals_->at(i).normal_z = map_->cloud->at(i).normal_z;
  }

  // NDT for coarse position search — wide basin, fast.
  ndt_ = pcl::make_shared<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
  ndt_->setResolution(1.0);   // 1m voxel — same as v14 (correct balance of basin/speed)
  ndt_->setStepSize(0.1);
  ndt_->setTransformationEpsilon(1e-3);
  ndt_->setMaximumIterations(50);
  ndt_->setInputTarget(map_xyz_);

  map_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  map_tree_->setInputCloud(map_xyz_);

  // small_gicp for GICP refinement.
  std::vector<Eigen::Vector3d> m_pts(map_xyz_->size());
  for (size_t i = 0; i < map_xyz_->size(); ++i)
    m_pts[i] = map_xyz_->at(i).getVector3fMap().cast<double>();
  auto [pc, tree] = small_gicp::preprocess_points(m_pts, 0.1, 10, 1);
  map_pc_ = pc;
  map_tree_sg_ = tree;
}

double FastGlobalMatcher::chamferScore(
    const pcl::PointCloud<pcl::PointXYZ>& query_aligned,
    double thresh) const {
  if (query_aligned.empty()) return 0.0;

  // Recall: query→map
  std::vector<int> idx(1);
  std::vector<float> dist2(1);
  double thresh2 = thresh * thresh;
  int recall = 0;
  for (const auto& p : query_aligned.points) {
    if (map_tree_->nearestKSearch(p, 1, idx, dist2) > 0 && dist2[0] <= thresh2)
      ++recall;
  }
  double r = static_cast<double>(recall) / query_aligned.size();

  // Precision: map→query (only map points within query footprint)
  pcl::KdTreeFLANN<pcl::PointXYZ> qtree;
  qtree.setInputCloud(query_aligned.makeShared());
  pcl::PointXYZ qmin, qmax;
  pcl::getMinMax3D(query_aligned, qmin, qmax);
  float margin = 0.5f;
  int precise = 0, m_total = 0;
  for (const auto& mp : map_xyz_->points) {
    if (mp.x < qmin.x - margin || mp.x > qmax.x + margin ||
        mp.y < qmin.y - margin || mp.y > qmax.y + margin) continue;
    ++m_total;
    if (qtree.nearestKSearch(mp, 1, idx, dist2) > 0 && dist2[0] <= thresh2)
      ++precise;
  }
  double p = m_total ? static_cast<double>(precise) / m_total : 0.0;

  return r * p;  // F0.5-like score (recall * precision)
}

FastGlobalMatcher::GicpResult FastGlobalMatcher::runGicp(
    const std::vector<Eigen::Vector3d>& query_pts,
    const Eigen::Isometry3d& init) const {
  auto [qpc, qtree] = small_gicp::preprocess_points(query_pts, 0.1, 10, 1);
  (void)qtree;

  small_gicp::RegistrationSetting s;
  s.type = small_gicp::RegistrationSetting::GICP;
  s.max_correspondence_distance = 1.5;
  s.max_iterations = 50;
  s.num_threads = 1;
  s.rotation_eps = 0.5 * M_PI / 180.0;
  s.translation_eps = 1e-3;
  small_gicp::RegistrationResult r =
      small_gicp::align(*map_pc_, *qpc, *map_tree_sg_, init, s);

  GicpResult gr;
  gr.pose = r.T_target_source;
  std::vector<int> nn_idx(1); std::vector<float> nn_dist2(1);
  int inliers = 0, tight = 0, n = 0;
  double sum = 0;
  for (const auto& p : query_pts) {
    pcl::PointXYZ q;
    Eigen::Vector3d tp = gr.pose * p;
    q.x = static_cast<float>(tp.x()); q.y = static_cast<float>(tp.y()); q.z = static_cast<float>(tp.z());
    if (map_tree_->nearestKSearch(q, 1, nn_idx, nn_dist2) > 0) {
      ++n;
      if (nn_dist2[0] <= 2.25) { ++inliers; sum += std::sqrt(nn_dist2[0]); if (nn_dist2[0] <= 0.09) ++tight; }
    }
  }
  gr.inlier = n ? static_cast<double>(inliers) / n : 0;
  gr.tight = n ? static_cast<double>(tight) / n : 0;
  gr.resid = inliers > 0 ? sum / inliers : 2.0;
  return gr;
}

std::vector<Candidate> FastGlobalMatcher::match(const pcl::PointCloud<pcl::PointXYZ>& query_raw) const {
  ensureBuilt();
  if (query_raw.empty()) return {};

  auto t0 = std::chrono::steady_clock::now();

  // Downsample query — coarse for NDT, fine for chamfer/GICP.
  auto q_ndt = voxelDown(query_raw, 0.5);       // 0.5m for NDT
  auto q_chamfer = voxelDown(query_raw, 0.5);   // 0.5m for chamfer sweep
  auto q_gicp = voxelDown(query_raw, params_.resolution);  // ~0.5m for GICP

  // Sensor z from query median.
  std::vector<float> qz(q_chamfer->size());
  for (size_t i = 0; i < q_chamfer->size(); ++i) qz[i] = q_chamfer->at(i).z;
  std::sort(qz.begin(), qz.end());
  double sensor_z = qz.empty() ? 0.0 : qz[qz.size() / 2];

  // ============ Stage 1: BEV coarse position search ============
  // BEV occupancy is fast (~1s) and gives position candidates. The yaw
  // ambiguity doesn't matter here — we just need rough positions.
  auto bev_cands = [&]() -> std::vector<std::pair<double,double>> {
    // Build a simple occupancy grid for the query (no yaw, just project XY).
    double bev_res = 1.0;
    std::set<std::pair<int,int>> q_cells;
    for (const auto& p : q_chamfer->points) {
      q_cells.insert({static_cast<int>(std::round(p.x / bev_res)),
                      static_cast<int>(std::round(p.y / bev_res))});
    }
    // Build map occupancy grid (already have map_xyz_).
    std::map<std::pair<int,int>, int> map_cells;
    for (const auto& p : map_xyz_->points) {
      auto key = std::make_pair(static_cast<int>(std::round(p.x / bev_res)),
                                static_cast<int>(std::round(p.y / bev_res)));
      map_cells[key]++;
    }
    // For each map position, count query occupancy overlap (cross-correlation).
    struct BevScored { double x, y; int overlap; };
    std::vector<BevScored> scored;
    for (double mx = map_->aabb.min.x(); mx <= map_->aabb.max.x(); mx += bev_res * 5) {
      for (double my = map_->aabb.min.y(); my <= map_->aabb.max.y(); my += bev_res * 5) {
        int overlap = 0;
        for (const auto& qc : q_cells) {
          auto key = std::make_pair(qc.first + static_cast<int>(std::round(mx / bev_res)),
                                    qc.second + static_cast<int>(std::round(my / bev_res)));
          auto it = map_cells.find(key);
          if (it != map_cells.end()) overlap += it->second;
        }
        scored.push_back({mx, my, overlap});
      }
    }
    std::sort(scored.begin(), scored.end(),
              [](const BevScored& a, const BevScored& b) { return a.overlap > b.overlap; });
    std::vector<std::pair<double,double>> out;
    for (const auto& s : scored) {
      bool dup = false;
      for (const auto& k : out) {
        if (std::hypot(k.first - s.x, k.second - s.y) < 5.0) { dup = true; break; }
      }
      if (!dup) out.push_back({s.x, s.y});
      if (static_cast<int>(out.size()) >= 5) break;
    }
    return out;
  };

  auto top_pos = bev_cands();

  auto t1 = std::chrono::steady_clock::now();

  // ============ Stage 2: Normal-consistency yaw sweep at each position ============
  // At each BEV position, sweep yaw 0-360° at 10° steps (36 steps).
  // For each yaw: transform query → find NN in map → compare normals.
  // Normal consistency IS yaw-discriminative at a FIXED position (unlike GICP
  // which moves the position to fit the wrong yaw).
  // Compute query normals once.
  pcl::PointCloud<pcl::Normal>::Ptr q_normals(new pcl::PointCloud<pcl::Normal>);
  {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(4);
    ne.setInputCloud(q_chamfer);
    ne.setRadiusSearch(0.5);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr ntree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(ntree);
    ne.compute(*q_normals);
  }

  struct YawScored { double x, y, z, yaw; double score; };
  std::vector<YawScored> yaw_results;

  const double yaw_step = 10.0 * M_PI / 180.0;
  const int n_yaw = 36;
  const double nn_thresh2 = 0.25;  // 0.5m

  // Sample query points for speed (every Nth).
  size_t q_stride = std::max<size_t>(1, q_chamfer->size() / 800);
  std::vector<size_t> q_sample;
  for (size_t i = 0; i < q_chamfer->size() && i < q_normals->size(); i += q_stride)
    q_sample.push_back(i);

  for (const auto& pos : top_pos) {
    for (int yi = 0; yi < n_yaw; ++yi) {
      double yaw = yi * yaw_step;
      double cy = std::cos(yaw), sy = std::sin(yaw);
      Eigen::Matrix3d R;
      R << cy, -sy, 0, sy, cy, 0, 0, 0, 1;

      int nc_good = 0, nc_total = 0;
      std::vector<int> nn_idx(1); std::vector<float> nn_dist2(1);
      for (size_t qi : q_sample) {
        const auto& p = q_chamfer->at(qi);
        const auto& n = q_normals->at(qi);
        if (!std::isfinite(n.normal_x) || n.normal_x == 0) continue;
        // Transform point + normal.
        double wx = cy * p.x - sy * p.y + pos.first;
        double wy = sy * p.x + cy * p.y + pos.second;
        double wz = p.z + sensor_z;
        pcl::PointXYZ qp; qp.x = wx; qp.y = wy; qp.z = wz;
        if (map_tree_->nearestKSearch(qp, 1, nn_idx, nn_dist2) > 0 && nn_dist2[0] <= nn_thresh2) {
          int mi = nn_idx[0];
          Eigen::Vector3d qn(R * Eigen::Vector3d(n.normal_x, n.normal_y, n.normal_z));
          Eigen::Vector3d mn(map_normals_->at(mi).normal_x, map_normals_->at(mi).normal_y, map_normals_->at(mi).normal_z);
          double dot = std::abs(qn.dot(mn));
          if (dot > 0.7) ++nc_good;
          ++nc_total;
        }
      }
      double score = nc_total ? static_cast<double>(nc_good) / nc_total : 0.0;
      yaw_results.push_back({pos.first, pos.second, sensor_z, yaw, score});}
  }

  auto t2 = std::chrono::steady_clock::now();

  // Sort by normal consistency score (higher = better).
  std::sort(yaw_results.begin(), yaw_results.end(),
            [](const YawScored& a, const YawScored& b) { return a.score > b.score; });

  // ============ Stage 3: GICP refine top-3 (position, yaw) ============
  std::vector<Eigen::Vector3d> q_pts(q_gicp->size());
  for (size_t i = 0; i < q_gicp->size(); ++i)
    q_pts[i] = q_gicp->at(i).getVector3fMap().cast<double>();

  int n_refine = std::min(3, static_cast<int>(yaw_results.size()));
  struct FinalScored { Eigen::Isometry3d pose; double tight; double resid; double inlier; };
  std::vector<FinalScored> finals;

  for (int i = 0; i < n_refine; ++i) {
    auto& yr = yaw_results[i];
    Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
    init.translation() = Eigen::Vector3d(yr.x, yr.y, yr.z);
    init.linear() = (Eigen::AngleAxisd(yr.yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    auto gr = runGicp(q_pts, init);
    finals.push_back({gr.pose, gr.tight, gr.resid, gr.inlier});
  }

  auto t3 = std::chrono::steady_clock::now();

  // Record timing.
  last_timing_ms[0] = std::chrono::duration<double, std::milli>(t1 - t0).count();
  last_timing_ms[1] = std::chrono::duration<double, std::milli>(t2 - t1).count();
  last_timing_ms[2] = std::chrono::duration<double, std::milli>(t3 - t2).count();
  last_timing_ms[3] = std::chrono::duration<double, std::milli>(t3 - t0).count();

  // Sort by tight (highest = best alignment).
  std::sort(finals.begin(), finals.end(),
            [](const FinalScored& a, const FinalScored& b) { return a.tight > b.tight; });

  // NMS + output.
  std::vector<Candidate> out;
  double dedup_t = params_.dedup_trans;
  for (const auto& s : finals) {
    bool dup = false;
    for (const auto& c : out) {
      if ((c.pose.translation() - s.pose.translation()).norm() < dedup_t) { dup = true; break; }
    }
    if (!dup) {
      Candidate c;
      c.pose = s.pose;
      c.inlier_ratio = s.inlier;
      c.tight_inlier_ratio = s.tight;
      c.overlap = s.inlier;
      c.mean_residual = s.resid;
      c.refined = true;
      c.score = std::clamp(s.tight, 0.0, 1.0);
      out.push_back(c);
    }
    if (static_cast<int>(out.size()) >= params_.top_k) break;
  }
  return out;
}

}  // namespace global_reloc
