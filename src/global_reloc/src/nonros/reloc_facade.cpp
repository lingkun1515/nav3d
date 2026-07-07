#include "global_reloc/nonros/reloc_facade.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

#include <random>
#include <cmath>
#include <stdexcept>

namespace global_reloc {

RelocFacade::RelocFacade(const std::string& params_path) {
  if (!params_path.empty()) {
    try { params_ = loadParamsFromFile(params_path); }
    catch (const std::exception&) { /* keep defaults */ }
  }
}

bool RelocFacade::buildMapFromPCD(const std::string& pcd_path,
                                  const std::string& out_dir,
                                  const std::string& name) {
  MapBuilder::BuildOptions opts;
  opts.feature = params_.feature;
  opts.voxel_size = build_voxel_;
  MapBuilder builder(opts);
  try {
    GlobalMap m = builder.buildFromPCD(pcd_path);
    if (!out_dir.empty()) saveGlobalMap(m, out_dir, name);
    map_ = std::make_shared<GlobalMap>(std::move(m));
    reloc_.reset();
    ensureRelocalizer();
    return true;
  } catch (const std::exception&) { return false; }
}

bool RelocFacade::loadMap(const std::string& gkey_path) {
  try {
    map_ = std::make_shared<GlobalMap>(loadGlobalMap(gkey_path));
    reloc_.reset();
    ensureRelocalizer();
    return true;
  } catch (const std::exception&) { return false; }
}

void RelocFacade::ensureRelocalizer() {
  if (!map_) return;
  reloc_ = std::make_unique<Relocalizer>(params_, map_);
  if (gravity_.squaredNorm() > 1e-6) reloc_->setGravityAttitude(gravity_);
}

RelocOutcome RelocFacade::estimate(const pcl::PointCloud<pcl::PointXYZ>& query) {
  RelocOutcome out;
  out.has_map = static_cast<bool>(map_);
  if (!map_ || !reloc_) return out;

  // Copy map points into the float buffer (downsampled view for the GUI).
  out.map_points.reserve(map_->cloud->size());
  for (const auto& p : map_->cloud->points) {
    out.map_points.emplace_back(p.x, p.y, p.z);
  }
  out.query_raw.reserve(query.size());
  for (const auto& p : query.points) {
    out.query_raw.emplace_back(p.x, p.y, p.z);
  }

  out.result = reloc_->estimateSingle(query);

  // Build aligned query.
  const Eigen::Matrix4f T = out.result.pose.matrix().cast<float>();
  out.query_aligned.reserve(query.size());
  for (const auto& p : query.points) {
    Eigen::Vector4f q(p.x, p.y, p.z, 1.0f);
    Eigen::Vector4f a = T * q;
    out.query_aligned.emplace_back(a.x(), a.y(), a.z());
  }
  return out;
}

// ---- synthetic helpers ----

pcl::PointCloud<pcl::PointXYZ> makeSyntheticRoom(int density) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::mt19937 rng(98765);
  auto addPlane = [&](const Eigen::Vector3d& o, const Eigen::Vector3d& u,
                      const Eigen::Vector3d& v, int n) {
    std::uniform_real_distribution<double> du(0, 1), dv(0, 1);
    for (int i = 0; i < n; ++i) {
      Eigen::Vector3d p = o + du(rng) * u + dv(rng) * v;
      pcl::PointXYZ pt; pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
      cloud.push_back(pt);
    }
  };
  double W = 8.0, H = 3.0;
  addPlane({0,0,0}, {W,0,0}, {0,W,0}, density/4);   // floor
  addPlane({0,0,H}, {W,0,0}, {0,W,0}, density/4);   // ceiling
  addPlane({0,0,0}, {W,0,0}, {0,0,H}, density/6);   // wall
  addPlane({0,W,0}, {W,0,0}, {0,0,H}, density/6);
  addPlane({0,0,0}, {0,W,0}, {0,0,H}, density/6);
  addPlane({W,0,0}, {0,W,0}, {0,0,H}, density/6);
  // two pillars for distinctive FPFH
  auto addPillar = [&](const Eigen::Vector3d& c, double r, int n) {
    std::uniform_real_distribution<double> da(0, 2*M_PI), dh(0, H);
    for (int i = 0; i < n; ++i) {
      double a = da(rng), z = dh(rng);
      pcl::PointXYZ pt; pt.x = c.x()+r*cos(a); pt.y = c.y()+r*sin(a); pt.z = z;
      cloud.push_back(pt);
    }
  };
  addPillar({2.0, 2.0, 0}, 0.2, density/3);
  addPillar({6.0, 6.0, 0}, 0.2, density/3);
  return cloud;
}

SyntheticQuery makeSyntheticQuery(const pcl::PointCloud<pcl::PointXYZ>& scene,
                                   const Eigen::Vector3d& viewpoint,
                                   double radius,
                                   const Eigen::Isometry3d& perturbation) {
  SyntheticQuery sq;
  for (const auto& p : scene.points) {
    if ((Eigen::Vector3d(p.x, p.y, p.z) - viewpoint).norm() <= radius) {
      pcl::PointXYZ pp; pp.x = p.x; pp.y = p.y; pp.z = p.z;
      sq.cloud.push_back(pp);
    }
  }
  // Apply perturbation: the query cloud is the scene perturbed by T.
  pcl::PointCloud<pcl::PointXYZ> perturbed;
  perturbed.reserve(sq.cloud.size());
  const Eigen::Matrix4f T = perturbation.matrix().cast<float>();
  for (const auto& p : sq.cloud.points) {
    Eigen::Vector4f q(p.x, p.y, p.z, 1.0f);
    Eigen::Vector4f a = T * q;
    pcl::PointXYZ np; np.x = a.x(); np.y = a.y(); np.z = a.z();
    perturbed.push_back(np);
  }
  sq.cloud = std::move(perturbed);
  // Ground-truth pose to recover is the inverse of the perturbation.
  sq.gt_pose = perturbation.inverse();
  return sq;
}

}  // namespace global_reloc
