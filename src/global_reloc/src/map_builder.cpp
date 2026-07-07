#include "global_reloc/map_builder.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/search.hpp>

#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <cstring>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace global_reloc {

namespace {
constexpr char kMagic[] = "GRELOC2";   // v2: optional intensity channel appended
constexpr char kMagicV1[] = "GRELOC1";
constexpr int kFpfhDim = 33;

// Minimal index file pointing to the PCD + fpfh bin + metadata.
void writeIndex(const std::string& path, const std::string& pcd_rel,
                const std::string& fpfh_rel, const std::string& inten_rel,
                size_t npoints, double voxel) {
  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot write index: " + path);
  size_t magic_len = std::strlen(kMagic);
  f.write(kMagic, magic_len);
  // PCD path length + bytes
  auto write_str = [&](const std::string& s) {
    uint32_t n = static_cast<uint32_t>(s.size());
    f.write(reinterpret_cast<const char*>(&n), sizeof(n));
    f.write(s.data(), n);
  };
  write_str(pcd_rel);
  write_str(fpfh_rel);
  write_str(inten_rel);   // v2: empty string if no intensity
  uint64_t n = static_cast<uint64_t>(npoints);
  f.write(reinterpret_cast<const char*>(&n), sizeof(n));
  double vx = voxel;
  f.write(reinterpret_cast<const char*>(&vx), sizeof(vx));
}

struct IndexFile {
  std::string pcd_rel;
  std::string fpfh_rel;
  std::string inten_rel;
  uint64_t npoints = 0;
  double voxel = 0;
};
IndexFile readIndex(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open index: " + path);
  char magic[8] = {0};
  f.read(magic, 7);
  bool v2 = (std::string(magic) == kMagic);
  bool v1 = (std::string(magic) == kMagicV1);
  if (!v1 && !v2) {
    throw std::runtime_error("bad map index magic: " + path);
  }
  IndexFile ix;
  auto read_str = [&](std::string& s) {
    uint32_t n = 0;
    f.read(reinterpret_cast<char*>(&n), sizeof(n));
    s.resize(n);
    if (n) f.read(&s[0], n);
  };
  read_str(ix.pcd_rel);
  read_str(ix.fpfh_rel);
  if (v2) read_str(ix.inten_rel);   // v1 has no intensity entry
  f.read(reinterpret_cast<char*>(&ix.npoints), sizeof(ix.npoints));
  f.read(reinterpret_cast<char*>(&ix.voxel), sizeof(ix.voxel));
  return ix;
}

std::string baseDir(const std::string& p) {
  size_t slash = p.find_last_of("/\\");
  return (slash == std::string::npos) ? "." : p.substr(0, slash);
}
std::string joinPath(const std::string& a, const std::string& b) {
  if (b.empty()) return a;
  if (!a.empty() && a.back() != '/' && a.back() != '\\') return a + "/" + b;
  return a + b;
}
}  // namespace

GlobalMap MapBuilder::buildFromPCD(const std::string& pcd_path) const {
  // Load as XYZI so reflectance (intensity) survives downsampling — it is the
  // key channel for disambiguating repetitive geometry later. Falls back to XYZ
  // (intensity 0) if the PCD has no intensity field.
  pcl::PointCloud<pcl::PointXYZI>::Ptr rawi(new pcl::PointCloud<pcl::PointXYZI>);
  bool has_intensity = (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *rawi) == 0);
  if (!has_intensity || rawi->empty()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr rawx(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *rawx) < 0 || rawx->empty()) {
      throw std::runtime_error("failed to load PCD: " + pcd_path);
    }
    rawi->resize(rawx->size());
    for (size_t i = 0; i < rawx->size(); ++i) {
      rawi->at(i).x = rawx->at(i).x; rawi->at(i).y = rawx->at(i).y;
      rawi->at(i).z = rawx->at(i).z; rawi->at(i).intensity = 0.f;
    }
    has_intensity = false;
  }
  std::vector<int> idx;
  pcl::removeNaNFromPointCloud(*rawi, *rawi, idx);

  GlobalMap m;
  m.voxel_size = opts_.voxel_size;

  // Voxel downsample XYZI (same voxel order as an XYZ downsample, so the
  // resulting intensity stays aligned with the xyz used for normals/FPFH).
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(rawi);
  vg.setLeafSize(opts_.voxel_size, opts_.voxel_size, opts_.voxel_size);
  pcl::PointCloud<pcl::PointXYZI>::Ptr downi(new pcl::PointCloud<pcl::PointXYZI>);
  vg.filter(*downi);

  // Normals on the downsampled xyz (computeNormals takes a PointXYZ cloud).
  pcl::PointCloud<pcl::PointXYZ>::Ptr downx(new pcl::PointCloud<pcl::PointXYZ>);
  downx->resize(downi->size());
  for (size_t i = 0; i < downi->size(); ++i) {
    downx->at(i).x = downi->at(i).x; downx->at(i).y = downi->at(i).y;
    downx->at(i).z = downi->at(i).z;
  }
  FeatureExtractor fe(opts_.feature);
  m.cloud = fe.computeNormals(*downx, 0.0);

  // Intensity parallel to m.cloud (computeNormals preserves input order at voxel=0).
  if (has_intensity && opts_.keep_intensity && m.cloud->size() == downi->size()) {
    m.intensity.resize(m.cloud->size());
    for (size_t i = 0; i < m.cloud->size(); ++i) m.intensity[i] = downi->at(i).intensity;
  }

  // FPFH on the downsampled RN cloud. (local var renamed to avoid collision
  // with pcl::fields::fpfh injected by the FPFHSignature33 registration macro.)
  m.fpfh.reset(new pcl::PointCloud<pcl::FPFHSignature33>);
  if (!m.cloud->empty()) {
    pcl::FPFHEstimationOMP<PointXYZRN, PointXYZRN, pcl::FPFHSignature33> fpfh_est;
#ifdef _OPENMP
    fpfh_est.setNumberOfThreads(opts_.feature.num_threads <= 0 ? omp_get_max_threads() : opts_.feature.num_threads);
#endif
    fpfh_est.setInputCloud(m.cloud);
    fpfh_est.setInputNormals(m.cloud);
    pcl::search::KdTree<PointXYZRN>::Ptr tree(new pcl::search::KdTree<PointXYZRN>);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(opts_.feature.fpfh_radius);
    fpfh_est.compute(*m.fpfh);
    if (m.fpfh->size() != m.cloud->size()) {
      throw std::runtime_error("map fpfh size mismatch");
    }
  }

  // AABB.
  if (!m.cloud->empty()) {
    PointXYZRN minp, maxp;
    pcl::getMinMax3D<PointXYZRN>(*m.cloud, minp, maxp);
    m.aabb.min = Eigen::Vector3d(minp.x, minp.y, minp.z);
    m.aabb.max = Eigen::Vector3d(maxp.x, maxp.y, maxp.z);
  }
  return m;
}

std::string MapBuilder::buildAndSave(const std::string& pcd_path,
                                     const std::string& out_dir,
                                     const std::string& name) const {
  GlobalMap m = buildFromPCD(pcd_path);
  return saveGlobalMap(m, out_dir, name);
}

std::string saveGlobalMap(const GlobalMap& m, const std::string& dir,
                          const std::string& name) {
  std::filesystem::create_directories(dir);
  std::string pcd_path = joinPath(dir, name + ".pcd");
  std::string fpfh_path = joinPath(dir, name + ".fpfh.bin");
  std::string inten_path = joinPath(dir, name + ".intensity.bin");
  std::string gkey_path = joinPath(dir, name + ".gkey");

  // Write the cloud (xyz+normal) as PCD binary.
  if (pcl::io::savePCDFileBinary(pcd_path, *m.cloud) < 0) {
    throw std::runtime_error("failed to write PCD: " + pcd_path);
  }
  // Write FPFH as raw float32 block: [n][33*n floats].
  {
    std::ofstream f(fpfh_path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot write fpfh: " + fpfh_path);
    uint64_t n = m.fpfh ? m.fpfh->size() : 0;
    f.write(reinterpret_cast<const char*>(&n), sizeof(n));
    if (n) {
      f.write(reinterpret_cast<const char*>(m.fpfh->points.data()),
              static_cast<std::streamsize>(n * kFpfhDim * sizeof(float)));
    }
  }
  // Write intensity (float32) if present.
  std::string inten_rel;
  if (m.hasIntensity()) {
    std::ofstream f(inten_path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot write intensity: " + inten_path);
    uint64_t n = m.intensity.size();
    f.write(reinterpret_cast<const char*>(&n), sizeof(n));
    if (n) f.write(reinterpret_cast<const char*>(m.intensity.data()),
                   static_cast<std::streamsize>(n * sizeof(float)));
    inten_rel = name + ".intensity.bin";
  }
  writeIndex(gkey_path, name + ".pcd", name + ".fpfh.bin", inten_rel,
             m.cloud ? m.cloud->size() : 0, m.voxel_size);
  return gkey_path;
}

GlobalMap loadGlobalMap(const std::string& gkey_path) {
  IndexFile ix = readIndex(gkey_path);
  std::string base = baseDir(gkey_path);

  GlobalMap m;
  m.voxel_size = ix.voxel;

  m.cloud.reset(new CloudRN);
  if (pcl::io::loadPCDFile<PointXYZRN>(joinPath(base, ix.pcd_rel), *m.cloud) < 0) {
    throw std::runtime_error("failed to load map PCD: " + ix.pcd_rel);
  }
  std::vector<int> idx;
  pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, idx);

  m.fpfh.reset(new pcl::PointCloud<pcl::FPFHSignature33>);
  std::ifstream f(joinPath(base, ix.fpfh_rel), std::ios::binary);
  if (f) {
    uint64_t n = 0;
    f.read(reinterpret_cast<char*>(&n), sizeof(n));
    if (n) {
      m.fpfh->resize(n);
      f.read(reinterpret_cast<char*>(m.fpfh->points.data()),
             static_cast<std::streamsize>(n * kFpfhDim * sizeof(float)));
    }
  }
  if (m.fpfh->size() != m.cloud->size()) {
    throw std::runtime_error("map fpfh size != cloud size after load");
  }

  // Load intensity if the index references it.
  if (!ix.inten_rel.empty()) {
    std::ifstream fi(joinPath(base, ix.inten_rel), std::ios::binary);
    if (fi) {
      uint64_t n = 0;
      fi.read(reinterpret_cast<char*>(&n), sizeof(n));
      if (n) {
        m.intensity.resize(n);
        fi.read(reinterpret_cast<char*>(m.intensity.data()),
                static_cast<std::streamsize>(n * sizeof(float)));
      }
    }
    if (m.intensity.size() != m.cloud->size()) m.intensity.clear();  // mismatch -> drop
  }

  // Recompute AABB from loaded cloud.
  if (!m.cloud->empty()) {
    PointXYZRN minp, maxp;
    pcl::getMinMax3D<PointXYZRN>(*m.cloud, minp, maxp);
    m.aabb.min = Eigen::Vector3d(minp.x, minp.y, minp.z);
    m.aabb.max = Eigen::Vector3d(maxp.x, maxp.y, maxp.z);
  }
  return m;
}

}  // namespace global_reloc
