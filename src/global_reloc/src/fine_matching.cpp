#include "global_reloc/fine_matching.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>

#include <stdexcept>

#ifdef GLOBAL_RELOC_HAVE_SMALL_GICP
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>
#endif

namespace global_reloc {

namespace {

// Downsample a raw XYZ cloud to the given voxel.
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

#ifdef GLOBAL_RELOC_HAVE_SMALL_GICP

small_gicp::RegistrationSetting toSmallGicpSetting(const FineParams& p) {
  small_gicp::RegistrationSetting s;
  // small_gicp has no "auto" mode; 0/<=0 -> single-threaded default for embedded.
  s.num_threads = (p.num_threads <= 0) ? 1 : p.num_threads;
  s.downsampling_resolution = p.voxel_size;
  s.max_correspondence_distance = p.max_correspondence_distance;
  s.max_iterations = p.max_iterations;
  // Map our single convergence_eps to both rotation/translation tolerances.
  s.rotation_eps = p.convergence_eps;
  s.translation_eps = p.convergence_eps;
  return s;
}

#endif  // GLOBAL_RELOC_HAVE_SMALL_GICP

// Compute inlier_ratio / overlap / mean_residual by NN query of transformed
// query against the map KdTree. Used by both backends. Also counts "tight"
// inliers (<= tight_thresh) — a sharp correctness signal that discriminates a
// genuine alignment (most points tight) from a plausible wrong basin in a
// dense/repetitive map. If reflectance (intensity) is available for both query
// and map, intensity_consistency = fraction of tight inliers whose reflectance
// matches within intensity_tol (0..255 normalized) — this breaks ties between
// repetitive geometric structures.
void evaluateOverlap(const pcl::PointCloud<pcl::PointXYZ>& transformed_query,
                     pcl::KdTreeFLANN<pcl::PointXYZ>& map_tree,
                     double max_corr, double tight_thresh, Candidate& c,
                     const std::vector<float>* q_inten = nullptr,
                     const std::vector<float>* map_inten = nullptr,
                     double intensity_tol = 0.2) {
  if (transformed_query.empty()) {
    c.inlier_ratio = 0; c.tight_inlier_ratio = 0; c.intensity_consistency = 0;
    c.num_inliers = 0; c.mean_residual = max_corr; c.overlap = 0;
    return;
  }
  int inliers = 0, tight = 0, inten_match = 0;
  double sum = 0;
  std::vector<int> idx(1);
  std::vector<float> dist(1);
  double tight2 = tight_thresh * tight_thresh;
  bool use_inten = (q_inten && map_inten &&
                    q_inten->size() == transformed_query.size() &&
                    !map_inten->empty());
  for (size_t i = 0; i < transformed_query.size(); ++i) {
    const auto& p = transformed_query.points[i];
    if (map_tree.nearestKSearch(p, 1, idx, dist) > 0) {
      double d2 = dist[0];
      if (d2 <= max_corr * max_corr) {
        ++inliers; sum += std::sqrt(d2);
        if (d2 <= tight2) {
          ++tight;
          if (use_inten) {
            float dI = std::abs((*q_inten)[i] - (*map_inten)[idx[0]]) / 255.0f;
            if (dI <= intensity_tol) ++inten_match;
          }
        }
      }
    }
  }
  size_t n = transformed_query.size();
  c.num_inliers = inliers;
  c.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(n);
  c.tight_inlier_ratio = static_cast<double>(tight) / static_cast<double>(n);
  c.intensity_consistency = (use_inten && tight > 0)
      ? static_cast<double>(inten_match) / static_cast<double>(tight) : 0.0;
  c.has_intensity = use_inten && tight > 0;
  c.mean_residual = inliers > 0 ? sum / inliers : max_corr;
  c.overlap = std::clamp(c.inlier_ratio, 0.0, 1.0);
}

}  // namespace

FineMatcher::FineMatcher(const FineParams& params) : params_(params) {
  if (params_.voxel_size <= 0) params_.voxel_size = 0.1;
  if (params_.max_correspondence_distance <= 0) params_.max_correspondence_distance = 1.0;
}

void FineMatcher::setMap(const GlobalMap* map) {
  if (!map) throw std::runtime_error("FineMatcher: null map");
  map_ = map;
}

void FineMatcher::refine(std::vector<Candidate>& candidates,
                         const pcl::PointCloud<pcl::PointXYZ>& query_raw,
                         const std::vector<float>* query_intensity) const {
  if (!map_) throw std::runtime_error("FineMatcher: map not set");
  if (map_->empty()) throw std::runtime_error("FineMatcher: empty map");

  // Downsample query once (shared across candidates and overlap eval). When the
  // caller supplied query reflectance, downsample as XYZI so the intensity stays
  // aligned with the downsampled xyz (VoxelGrid preserves a deterministic order).
  pcl::PointCloud<pcl::PointXYZ>::Ptr qdown;
  std::vector<float> qdown_inten;
  bool have_qi = (query_intensity && !query_intensity->empty() &&
                  query_intensity->size() == query_raw.size() && map_->hasIntensity());
  if (have_qi) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr qi(new pcl::PointCloud<pcl::PointXYZI>);
    qi->resize(query_raw.size());
    for (size_t i = 0; i < query_raw.size(); ++i) {
      qi->at(i).x = query_raw.at(i).x;
      qi->at(i).y = query_raw.at(i).y;
      qi->at(i).z = query_raw.at(i).z;
      qi->at(i).intensity = (*query_intensity)[i];
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr qdi(new pcl::PointCloud<pcl::PointXYZI>);
    if (params_.voxel_size > 0) {
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(qi);
      vg.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
      vg.filter(*qdi);
    } else {
      *qdi = *qi;
    }
    qdown.reset(new pcl::PointCloud<pcl::PointXYZ>);
    qdown->resize(qdi->size());
    qdown_inten.resize(qdi->size());
    for (size_t i = 0; i < qdi->size(); ++i) {
      qdown->at(i).x = qdi->at(i).x; qdown->at(i).y = qdi->at(i).y;
      qdown->at(i).z = qdi->at(i).z;
      qdown_inten[i] = qdi->at(i).intensity;
    }
  } else {
    qdown = voxelDown(query_raw, params_.voxel_size);
  }
  const std::vector<float>* q_inten_ptr = have_qi ? &qdown_inten : nullptr;
  const std::vector<float>* map_inten_ptr = map_->hasIntensity() ? &map_->intensity : nullptr;

  // Build a PCL KdTree over the map for overlap evaluation (both backends).
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  map_xyz->resize(map_->cloud->size());
  for (size_t i = 0; i < map_->cloud->size(); ++i) {
    map_xyz->at(i).x = map_->cloud->at(i).x;
    map_xyz->at(i).y = map_->cloud->at(i).y;
    map_xyz->at(i).z = map_->cloud->at(i).z;
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> map_tree;
  map_tree.setInputCloud(map_xyz);

#ifdef GLOBAL_RELOC_HAVE_SMALL_GICP
  // Preprocess map + query ONCE (downsample+kdtree+normals/cov), then reuse
  // across all candidates — avoids top_k-fold redundant map preprocessing.
  int nt = (params_.num_threads <= 0) ? 1 : params_.num_threads;
  std::vector<Eigen::Vector3d> map_pts(map_xyz->size());
  for (size_t i = 0; i < map_xyz->size(); ++i)
    map_pts[i] = map_xyz->at(i).getVector3fMap().cast<double>();
  std::vector<Eigen::Vector3d> q_pts(qdown->size());
  for (size_t i = 0; i < qdown->size(); ++i)
    q_pts[i] = qdown->at(i).getVector3fMap().cast<double>();

  auto [map_pc, map_kd] = small_gicp::preprocess_points(
      map_pts, params_.voxel_size, 10, nt);
  auto [query_pc, query_kd] = small_gicp::preprocess_points(
      q_pts, params_.voxel_size, 10, nt);
  (void)query_kd;  // query tree not needed for the align overload
  auto setting = toSmallGicpSetting(params_);
  // Preprocessed-cloud overload ignores downsampling_resolution (already done);
  // it uses rotation_eps/translation_eps/max_correspondence_distance/max_iterations.

  for (auto& cand : candidates) {
    small_gicp::RegistrationResult r =
        small_gicp::align(*map_pc, *query_pc, *map_kd, cand.pose, setting);
    cand.pose = r.T_target_source;
    cand.refined = true;

    pcl::PointCloud<pcl::PointXYZ> tf_q;
    pcl::transformPointCloud(*qdown, tf_q, cand.pose.matrix().cast<float>());
    evaluateOverlap(tf_q, map_tree, params_.max_correspondence_distance,
                    params_.tight_threshold, cand,
                    q_inten_ptr, map_inten_ptr, params_.intensity_tol);
  }
#else
  // Fallback: PCL GeneralizedIterativeClosestPoint.
  for (auto& cand : candidates) {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(qdown);
    gicp.setInputTarget(map_xyz);
    gicp.setMaxCorrespondenceDistance(params_.max_correspondence_distance);
    gicp.setMaximumIterations(params_.max_iterations);
    gicp.setTransformationEpsilon(params_.convergence_eps);
    gicp.setEuclideanFitnessEpsilon(params_.convergence_eps);

    Eigen::Matrix4f guess = cand.pose.matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZ> aligned;
    gicp.align(aligned, guess);
    cand.pose = Eigen::Isometry3d(gicp.getFinalTransformation().cast<double>());
    cand.refined = true;
    evaluateOverlap(aligned, map_tree, params_.max_correspondence_distance,
                    params_.tight_threshold, cand,
                    q_inten_ptr, map_inten_ptr, params_.intensity_tol);
  }
#endif
}

}  // namespace global_reloc
