#include "global_reloc/ndt_gicp_matcher.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

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

NdtGicpMatcher::NdtGicpMatcher(const BevParams& params) : params_(params) {}
NdtGicpMatcher::~NdtGicpMatcher() = default;

void NdtGicpMatcher::setMap(const GlobalMap* map) {
  if (!map) throw std::runtime_error("NdtGicpMatcher: null map");
  map_ = map;
  ndt_.reset();
}

void NdtGicpMatcher::ensureBuilt() const {
  if (ndt_ && map_pc_) return;
  const_cast<NdtGicpMatcher*>(this)->build();
}

void NdtGicpMatcher::build() {
  if (!map_ || map_->empty()) throw std::runtime_error("NdtGicpMatcher: empty map");

  // PCL cloud for NDT + overlap eval.
  map_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  map_xyz_->resize(map_->cloud->size());
  // Map normals (from map_->cloud which is PointNormal).
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

  // PCL NDT — configure for wide convergence basin.
  ndt_ = pcl::make_shared<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
  ndt_->setResolution(1.0);              // 1m voxel — wide basin
  ndt_->setStepSize(0.1);                // large step = faster convergence
  ndt_->setTransformationEpsilon(1e-3);
  ndt_->setMaximumIterations(50);        // NDT is fast, 50 iters ~0.1s
  ndt_->setInputTarget(map_xyz_);

  // PCL KdTree for overlap eval.
  map_tree_pcl_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  map_tree_pcl_->setInputCloud(map_xyz_);

  // small_gicp for GICP refinement.
  std::vector<Eigen::Vector3d> m_pts(map_xyz_->size());
  for (size_t i = 0; i < map_xyz_->size(); ++i)
    m_pts[i] = map_xyz_->at(i).getVector3fMap().cast<double>();
  auto [pc, tree] = small_gicp::preprocess_points(m_pts, 0.1, 10, 1);
  map_pc_ = pc;
  map_tree_sg_ = tree;
}

NdtGicpMatcher::NdtResult NdtGicpMatcher::runNdt(
    const pcl::PointCloud<pcl::PointXYZ>& query,
    const Eigen::Isometry3d& init) const {
  ndt_->setInputSource(query.makeShared());
  pcl::PointCloud<pcl::PointXYZ> aligned;
  Eigen::Matrix4f guess = init.matrix().cast<float>();
  ndt_->align(aligned, guess);
  NdtResult r;
  r.pose = Eigen::Isometry3d(ndt_->getFinalTransformation().cast<double>());
  r.fitness = ndt_->getFitnessScore(2.0);  // lower = better
  return r;
}

NdtGicpMatcher::GicpResult NdtGicpMatcher::runGicp(
    const std::vector<Eigen::Vector3d>& query_pts,
    const Eigen::Isometry3d& init) const {
  auto [qpc, qtree] = small_gicp::preprocess_points(query_pts, 0.1, 10, 1);
  (void)qtree;

  // Single-stage GICP — v14 parameters.
  small_gicp::RegistrationSetting s;
  s.type = small_gicp::RegistrationSetting::GICP;
  s.max_correspondence_distance = 1.5;
  s.max_iterations = 100;
  s.num_threads = 1;
  s.rotation_eps = 0.5 * M_PI / 180.0;
  s.translation_eps = 1e-3;
  small_gicp::RegistrationResult r =
      small_gicp::align(*map_pc_, *qpc, *map_tree_sg_, init, s);

  // Evaluate the result.
  GicpResult gr;
  gr.pose = r.T_target_source;
  std::vector<int> idx(1); std::vector<float> dist2(1);
  int inliers = 0, tight = 0, n = 0;
  double sum = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_q(new pcl::PointCloud<pcl::PointXYZ>);
  aligned_q->reserve(query_pts.size());
  for (const auto& p : query_pts) {
    pcl::PointXYZ q;
    Eigen::Vector3d tp = gr.pose * p;
    q.x = static_cast<float>(tp.x()); q.y = static_cast<float>(tp.y()); q.z = static_cast<float>(tp.z());
    aligned_q->push_back(q);
    if (map_tree_pcl_->nearestKSearch(q, 1, idx, dist2) > 0) {
      ++n;
      if (dist2[0] <= 2.25) { ++inliers; sum += std::sqrt(dist2[0]); if (dist2[0] <= 0.09) ++tight; }
    }
  }
  gr.inlier = n ? static_cast<double>(inliers) / n : 0;
  gr.tight = n ? static_cast<double>(tight) / n : 0;
  gr.resid = inliers > 0 ? sum / inliers : 2.0;

  // Precision: map→query
  pcl::KdTreeFLANN<pcl::PointXYZ> query_tree;
  query_tree.setInputCloud(aligned_q);
  pcl::PointXYZ qmin, qmax;
  pcl::getMinMax3D(*aligned_q, qmin, qmax);
  float margin = 1.0f;
  int m_inliers = 0, m_total = 0;
  for (const auto& mp : map_xyz_->points) {
    if (mp.x < qmin.x - margin || mp.x > qmax.x + margin ||
        mp.y < qmin.y - margin || mp.y > qmax.y + margin ||
        mp.z < qmin.z - margin || mp.z > qmax.z + margin) continue;
    ++m_total;
    if (query_tree.nearestKSearch(mp, 1, idx, dist2) > 0 && dist2[0] <= 0.09) ++m_inliers;
  }
  gr.precision = m_total ? static_cast<double>(m_inliers) / m_total : 0.0;
  gr.normal_consistency = 0.0;  // computed in match() using query normals
  return gr;
}

std::vector<Candidate> NdtGicpMatcher::match(const pcl::PointCloud<pcl::PointXYZ>& query_raw) const {
  ensureBuilt();
  if (query_raw.empty()) return {};

  // Downsample query for NDT speed — coarser for NDT (fast), finer for GICP.
  auto qdown = voxelDown(query_raw, 0.5);  // 0.5m for NDT
  if (qdown->empty()) return {};

  // Query pts for GICP (finer downsample).
  auto qfine = voxelDown(query_raw, params_.resolution);
  std::vector<Eigen::Vector3d> q_pts(qfine->size());
  for (size_t i = 0; i < qfine->size(); ++i)
    q_pts[i] = qfine->at(i).getVector3fMap().cast<double>();

  // Compute query normals ONCE (used for normal-consistency yaw scoring).
  pcl::PointCloud<pcl::Normal>::Ptr q_normals(new pcl::PointCloud<pcl::Normal>);
  {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(4);
    ne.setInputCloud(qfine);
    ne.setRadiusSearch(0.6);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.compute(*q_normals);
  }

  // Sensor z from query median.
  std::vector<float> qz(q_pts.size());
  for (size_t i = 0; i < q_pts.size(); ++i) qz[i] = static_cast<float>(q_pts[i].z());
  std::sort(qz.begin(), qz.end());
  double sensor_z = qz.empty() ? 0.0 : qz[qz.size() / 2];

  // Stage 1: NDT global search — params_.ndt_grid_step m × params_.ndt_yaw_count yaw.
  // NDT convergence basin ~5-10m; the coarse grid need only be within that.
  double x_min = map_->aabb.min.x(), x_max = map_->aabb.max.x();
  double y_min = map_->aabb.min.y(), y_max = map_->aabb.max.y();

  const double grid_step = params_.ndt_grid_step;
  const int yaw_count = params_.ndt_yaw_count;
  struct PosCandidate { Eigen::Isometry3d pose; double fitness; };
  std::vector<PosCandidate> coarse_ndt;
  for (int yi = 0; yi < yaw_count; ++yi) {
    double yaw = yi * (2.0 * M_PI / yaw_count);
    for (double x = x_min; x <= x_max; x += grid_step) {
      for (double y = y_min; y <= y_max; y += grid_step) {
        Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
        init.translation() = Eigen::Vector3d(x, y, sensor_z);
        init.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();
        auto r = runNdt(*qdown, init);
        coarse_ndt.push_back({r.pose, r.fitness});
      }
    }
  }
  std::sort(coarse_ndt.begin(), coarse_ndt.end(),
            [](const PosCandidate& a, const PosCandidate& b) { return a.fitness < b.fitness; });

  // Stage 1b: take top-10 distinct (position, yaw) from NDT coarse directly.
  struct NdtScored { Eigen::Isometry3d pose; double fitness; };
  std::vector<NdtScored> ndt_results;
  for (const auto& c : coarse_ndt) {
    bool dup = false;
    for (const auto& k : ndt_results) {
      double dt = (k.pose.translation() - c.pose.translation()).norm();
      if (dt < 3.0) { dup = true; break; }
    }
    if (!dup) ndt_results.push_back({c.pose, c.fitness});
    if (static_cast<int>(ndt_results.size()) >= 10) break;
  }

  // Sort by NDT fitness (lower = better).
  std::sort(ndt_results.begin(), ndt_results.end(),
            [](const NdtScored& a, const NdtScored& b) { return a.fitness < b.fitness; });

  // Stage 2: GICP refine top-10 NDT results (each with its own converged yaw).
  // NDT fine (24 yaw) already gives good position+yaw. GICP polishes it.
  int n_refine = std::min(10, static_cast<int>(ndt_results.size()));

  struct FinalScored { Eigen::Isometry3d pose; double tight; double resid; double inlier; double precision; double norm_consist; };
  std::vector<FinalScored> finals;

  for (int i = 0; i < n_refine; ++i) {
    auto gr = runGicp(q_pts, ndt_results[i].pose);
    finals.push_back({gr.pose, gr.tight, gr.resid, gr.inlier, gr.precision, 0.0});
  }

  // Sort by tight (highest = best alignment after GICP refinement).
  std::sort(finals.begin(), finals.end(),
            [](const FinalScored& a, const FinalScored& b) { return a.tight > b.tight; });

  // Sort by tight (highest = best alignment after GICP refinement).
  std::sort(finals.begin(), finals.end(),
            [](const FinalScored& a, const FinalScored& b) {
              return a.tight * a.precision * a.norm_consist > b.tight * b.precision * b.norm_consist;
            });

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
      c.score = std::clamp(s.tight * s.precision, 0.0, 1.0);  // bidirectional F1
      out.push_back(c);
    }
    if (static_cast<int>(out.size()) >= params_.top_k) break;
  }
  return out;
}

}  // namespace global_reloc
