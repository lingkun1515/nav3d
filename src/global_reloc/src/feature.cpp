#include "global_reloc/feature.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace global_reloc {

FeatureExtractor::FeatureExtractor(const FeatureParams& params) : params_(params) {
  if (params_.normal_radius <= 0 || params_.fpfh_radius <= 0) {
    throw std::runtime_error("FeatureParams radii must be positive");
  }
}

int FeatureExtractor::resolvedThreads() const {
#ifdef _OPENMP
  if (params_.num_threads <= 0) return omp_get_max_threads();
  return params_.num_threads;
#else
  return 1;
#endif
}

CloudRNPtr FeatureExtractor::computeNormals(const pcl::PointCloud<pcl::PointXYZ>& raw,
                                           double voxel_size) const {
  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
  if (voxel_size > 0) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(raw.makeShared());
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*down);
  } else {
    *down = raw;
  }

  CloudRNPtr out(new CloudRN);
  // Copy xyz into the RN cloud (normals zeroed).
  pcl::copyPointCloud(*down, *out);

  if (out->empty()) return out;

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
  ne.setNumberOfThreads(resolvedThreads());
  ne.setInputCloud(down);
  ne.setRadiusSearch(params_.normal_radius);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
  ne.compute(*normals);

  if (normals->size() != out->size()) {
    throw std::runtime_error("normal estimation size mismatch");
  }
  for (size_t i = 0; i < out->size(); ++i) {
    out->at(i).normal_x = normals->at(i).normal_x;
    out->at(i).normal_y = normals->at(i).normal_y;
    out->at(i).normal_z = normals->at(i).normal_z;
    out->at(i).curvature = normals->at(i).curvature;
  }
  return out;
}

FeatureCloud FeatureExtractor::extract(const pcl::PointCloud<pcl::PointXYZ>& raw,
                                       double voxel_size_override) const {
  FeatureCloud fc;
  // Step 1: voxel downsample + normals (reuse computeNormals path).
  const double voxel = (voxel_size_override > 0) ? voxel_size_override : params_.normal_radius * 0.5;
  fc.cloud = computeNormals(raw, voxel);

  fc.fpfh.reset(new pcl::PointCloud<pcl::FPFHSignature33>);
  if (fc.cloud->empty()) return fc;

  // Step 2: FPFH. Need a XYZRN cloud already has normals; feed a PointXYZ cloud
  // that shares xyz with fc.cloud and let FPFH read normals via the RN cloud.
  // FPFHEstimationOMP expects input + normals; we pass fc.cloud as the input
  // (XYZRN is a custom point type — use a dedicated estimation instance).
  pcl::FPFHEstimationOMP<PointXYZRN, PointXYZRN, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setNumberOfThreads(resolvedThreads());
  fpfh_est.setInputCloud(fc.cloud);
  fpfh_est.setInputNormals(fc.cloud);
  pcl::search::KdTree<PointXYZRN>::Ptr tree(new pcl::search::KdTree<PointXYZRN>);
  fpfh_est.setSearchMethod(tree);
  fpfh_est.setRadiusSearch(params_.fpfh_radius);
  fpfh_est.compute(*fc.fpfh);

  if (fc.fpfh->size() != fc.cloud->size()) {
    throw std::runtime_error("fpfh estimation size mismatch");
  }
  return fc;
}

}  // namespace global_reloc
