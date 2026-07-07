#include "global_reloc/ndt_matching.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

#include <cmath>
#include <algorithm>

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

NdtMatcher::NdtMatcher(const BevParams& params) : params_(params) {}
NdtMatcher::~NdtMatcher() = default;

void NdtMatcher::setMap(const GlobalMap* map) {
  if (!map) throw std::runtime_error("NdtMatcher: null map");
  map_ = map;
  map_pc_.reset();
  map_tree_.reset();
}

void NdtMatcher::ensureBuilt() const {
  if (map_pc_ && map_tree_ && map_tree_pcl_) return;
  const_cast<NdtMatcher*>(this)->build();
}

void NdtMatcher::build() {
  if (!map_ || map_->empty()) throw std::runtime_error("NdtMatcher: empty map");

  // PCL cloud for overlap eval.
  map_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  map_xyz_->resize(map_->cloud->size());
  for (size_t i = 0; i < map_->cloud->size(); ++i) {
    map_xyz_->at(i).x = map_->cloud->at(i).x;
    map_xyz_->at(i).y = map_->cloud->at(i).y;
    map_xyz_->at(i).z = map_->cloud->at(i).z;
  }

  // small_gicp point cloud + kdtree (preprocessed once, reused across yaws).
  // CRITICAL: downsampling_resolution must be > 0. 0.0 collapses the cloud
  // to ~1 point, making all GICP runs meaningless. Use 0.1 (map already 0.4m
  // downsampled; 0.1 keeps detail without collapsing).
  std::vector<Eigen::Vector3d> map_pts(map_xyz_->size());
  for (size_t i = 0; i < map_xyz_->size(); ++i)
    map_pts[i] = map_xyz_->at(i).getVector3fMap().cast<double>();
  auto [pc, tree] = small_gicp::preprocess_points(map_pts, 0.1, 10, 1);
  map_pc_ = pc;
  map_tree_ = tree;

  map_tree_pcl_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  map_tree_pcl_->setInputCloud(map_xyz_);
}

NdtMatcher::GicpResult NdtMatcher::runGicp(
    const std::vector<Eigen::Vector3d>& query_pts,
    const Eigen::Isometry3d& init_T) const {
  // Preprocess query with proper downsampling (0.0 collapses to 1 point!).
  auto [qpc, qtree] = small_gicp::preprocess_points(query_pts, 0.1, 10, 1);
  (void)qtree;

  small_gicp::RegistrationSetting s;
  s.type = small_gicp::RegistrationSetting::GICP;
  s.max_correspondence_distance = 5.0;  // wide basin for global search
  s.max_iterations = 50;
  s.num_threads = 1;
  s.rotation_eps = 0.5 * M_PI / 180.0;
  s.translation_eps = 1e-3;

  small_gicp::RegistrationResult r =
      small_gicp::align(*map_pc_, *qpc, *map_tree_, init_T, s);

  // Evaluate overlap with PCL KdTree (query→map NN inlier ratio).
  GicpResult gr;
  gr.pose = r.T_target_source;
  std::vector<int> idx(1);
  std::vector<float> dist2(1);
  int inliers = 0, tight = 0;
  double sum = 0;
  double max_corr = 2.0, tight_thr = 0.3;
  double max2 = max_corr * max_corr, tight2 = tight_thr * tight_thr;
  int n = 0;
  for (const auto& p : query_pts) {
    pcl::PointXYZ q;
    Eigen::Vector3d tp = gr.pose * p;
    q.x = static_cast<float>(tp.x());
    q.y = static_cast<float>(tp.y());
    q.z = static_cast<float>(tp.z());
    if (map_tree_pcl_->nearestKSearch(q, 1, idx, dist2) > 0) {
      ++n;
      if (dist2[0] <= max2) {
        ++inliers; sum += std::sqrt(dist2[0]);
        if (dist2[0] <= tight2) ++tight;
      }
    }
  }
  gr.inlier_ratio = n ? static_cast<double>(inliers) / n : 0;
  gr.tight_ratio = n ? static_cast<double>(tight) / n : 0;
  gr.resid = inliers > 0 ? sum / inliers : max_corr;
  gr.E = r.error;       // GICP registration cost (lower = better fit)
  gr.converged = r.converged;
  return gr;
}

std::vector<Candidate> NdtMatcher::match(const pcl::PointCloud<pcl::PointXYZ>& query_raw) const {
  return matchFromCandidates(query_raw, {});
}

std::vector<Candidate> NdtMatcher::matchFromCandidates(
    const pcl::PointCloud<pcl::PointXYZ>& query_raw,
    const std::vector<Candidate>& bev_candidates) const {
  ensureBuilt();
  if (query_raw.empty()) return {};

  // Downsample query for speed (GICP is the bottleneck).
  auto qdown = voxelDown(query_raw, params_.resolution);
  if (qdown->empty()) return {};

  std::vector<Eigen::Vector3d> q_pts(qdown->size());
  for (size_t i = 0; i < qdown->size(); ++i)
    q_pts[i] = qdown->at(i).getVector3fMap().cast<double>();

  // Estimate sensor z from query median (robot on ground).
  std::vector<float> qz(q_pts.size());
  for (size_t i = 0; i < q_pts.size(); ++i) qz[i] = static_cast<float>(q_pts[i].z());
  std::sort(qz.begin(), qz.end());
  double sensor_z = qz.empty() ? 0.0 : qz[qz.size() / 2];

  struct Scored { double yaw; Eigen::Isometry3d pose; double tight; double resid; double inlier; double E; bool converged; };
  std::vector<Scored> results;

  // If BEV candidates are provided, use them as GICP init positions (fast:
  // ~10 candidates × 8 yaw = 80 runs instead of ~3700). Otherwise fall back
  // to the dense 5m grid (slower but zero-prior).
  std::vector<std::pair<double,double>> init_pos;
  if (!bev_candidates.empty()) {
    for (const auto& c : bev_candidates) {
      init_pos.push_back({c.pose.translation().x(), c.pose.translation().y()});
    }
  } else {
    // Dense 5m grid (within GICP's ~3m basin; 5m worst-case offset ~2.5m).
    double x_min = map_->aabb.min.x(), x_max = map_->aabb.max.x();
    double y_min = map_->aabb.min.y(), y_max = map_->aabb.max.y();
    for (double x = x_min; x <= x_max; x += 5.0)
      for (double y = y_min; y <= y_max; y += 5.0)
        init_pos.push_back({x, y});
  }

  // 8 yaw angles (every 45°).
  const int n_yaw = 8;
  const double yaw_step = 45.0 * M_PI / 180.0;

  for (int yi = 0; yi < n_yaw; ++yi) {
    double yaw = yaw_step * yi;
    for (const auto& pos : init_pos) {
      Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
      init.translation() = Eigen::Vector3d(pos.first, pos.second, sensor_z);
      init.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();
      auto r = runGicp(q_pts, init);
      results.push_back({yaw, r.pose, r.tight_ratio, r.resid, r.inlier_ratio, r.E, r.converged});
    }
  }

  // Sort by tight inlier ratio (highest = best alignment). No refine step —
  // the dense 3m grid + 8 yaw already covers the convergence basin.
  std::sort(results.begin(), results.end(),
            [](const Scored& a, const Scored& b) { return a.tight > b.tight; });

  // NMS: dedup by translation proximity.
  std::vector<Scored> kept;
  double dedup_t = params_.dedup_trans;
  for (const auto& s : results) {
    bool dup = false;
    for (const auto& k : kept) {
      double dt = (s.pose.translation() - k.pose.translation()).norm();
      if (dt < dedup_t) { dup = true; break; }
    }
    if (!dup) kept.push_back(s);
    if (static_cast<int>(kept.size()) >= params_.top_k) break;
  }

  std::vector<Candidate> out;
  out.reserve(kept.size());
  for (const auto& s : kept) {
    Candidate c;
    c.pose = s.pose;
    c.inlier_ratio = s.inlier;
    c.tight_inlier_ratio = s.tight;
    c.overlap = s.inlier;
    c.mean_residual = s.resid;
    c.refined = true;  // already GICP-refined
    c.score = std::clamp(s.tight, 0.0, 1.0);  // tight inlier as score
    out.push_back(c);
  }
  return out;
}

}  // namespace global_reloc
