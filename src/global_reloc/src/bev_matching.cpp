#include "global_reloc/bev_matching.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

#include <cmath>
#include <climits>
#include <algorithm>
#include <stdexcept>
#include <map>

namespace global_reloc {

namespace {

constexpr float kInf = 1e20f;

// 1D squared-distance transform of a sampled function f over [0, n).
// d[x] = min_x' ((x - x')^2 + f[x']).  Felzenszwalb & Huttenlocher (2012).
void dt1d(const float* f, float* d, int n, std::vector<int>& v, std::vector<float>& z) {
  v.assign(n, 0);
  z.assign(n + 1, 0.0f);
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = +kInf;
  for (int q = 1; q < n; ++q) {
    float fq = f[q] + static_cast<float>(q * q);
    float fv = f[v[k]] + static_cast<float>(v[k] * v[k]);
    float s = (fq - fv) / (2.0f * static_cast<float>(q - v[k]));
    while (s <= z[k]) {
      --k;
      fq = f[q] + static_cast<float>(q * q);
      fv = f[v[k]] + static_cast<float>(v[k] * v[k]);
      s = (fq - fv) / (2.0f * static_cast<float>(q - v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = +kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<float>(q)) ++k;
    int diff = q - v[k];
    d[q] = static_cast<float>(diff * diff) + f[v[k]];
  }
}

// Exact 2D Euclidean distance transform of a binary occupancy grid.
// Returns squared distance (in cell units) to the nearest occupied cell.
// Occupied cells (occ[i] == true) get 0.
std::vector<float> edt2d(const std::vector<char>& occ, int W, int H) {
  std::vector<float> tmp(W * H);
  std::vector<float> out(W * H);
  std::vector<int> v;
  std::vector<float> z;

  std::vector<float> f(std::max(W, H));
  std::vector<float> d(std::max(W, H));

  // Pass 1: transform along X for each row.
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) f[x] = occ[y * W + x] ? 0.0f : kInf;
    dt1d(f.data(), d.data(), W, v, z);
    for (int x = 0; x < W; ++x) tmp[y * W + x] = d[x];
  }
  // Pass 2: transform along Y for each column, using tmp as the function.
  for (int x = 0; x < W; ++x) {
    for (int y = 0; y < H; ++y) f[y] = tmp[y * W + x];
    dt1d(f.data(), d.data(), H, v, z);
    for (int y = 0; y < H; ++y) out[y * W + x] = d[y];
  }
  return out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownXYZ(
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

BevMatcher::BevMatcher(const BevParams& params) : params_(params) {
  if (params_.resolution <= 0) params_.resolution = 0.5;
  if (params_.sigma <= 0) params_.sigma = 0.35;
  if (params_.yaw_step_deg <= 0) params_.yaw_step_deg = 2.0;
  if (params_.top_k <= 0) params_.top_k = 1;
}

void BevMatcher::setMap(const GlobalMap* map) {
  if (!map) throw std::runtime_error("BevMatcher: null map");
  map_ = map;
  field_.reset();
  map_xyz_.reset();
  map_tree_.reset();
}

void BevMatcher::ensureBuilt() const {
  if (field_ && map_tree_) return;
  // Defer to a non-const build (const method -> mutable cache via const_cast).
  const_cast<BevMatcher*>(this)->buildField();
}

void BevMatcher::buildField() {
  if (!map_ || map_->empty()) throw std::runtime_error("BevMatcher: empty map");

  // Copy map XYZ into a plain PointXYZ cloud (map cloud is xyz+normal).
  map_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  map_xyz_->resize(map_->cloud->size());
  for (size_t i = 0; i < map_->cloud->size(); ++i) {
    map_xyz_->at(i).x = map_->cloud->at(i).x;
    map_xyz_->at(i).y = map_->cloud->at(i).y;
    map_xyz_->at(i).z = map_->cloud->at(i).z;
  }
  // Copy intensity if the map carries it — the key disambiguation channel.
  map_inten_.clear();
  if (map_->hasIntensity()) {
    map_inten_ = map_->intensity;
  }

  const double res = params_.resolution;
  field_ = std::make_shared<Field>();
  field_->resolution = res;
  field_->origin_x = map_->aabb.min.x();
  field_->origin_y = map_->aabb.min.y();
  const double ext_x = map_->aabb.max.x() - map_->aabb.min.x();
  const double ext_y = map_->aabb.max.y() - map_->aabb.min.y();
  field_->W = static_cast<int>(std::floor(ext_x / res)) + 1;
  field_->H = static_cast<int>(std::floor(ext_y / res)) + 1;
  if (field_->W <= 0 || field_->H <= 0) throw std::runtime_error("BevMatcher: bad map extent");

  // Rasterize map points into occupancy + per-cell max-z (height channel).
  // max-z lets the coarse stage discriminate BEV-identical structures at
  // different heights — the dominant failure mode (ceiling vs floor, two
  // corridor levels). Occupancy alone is ambiguous; height breaks ties.
  std::vector<char> occ(field_->W * field_->H, 0);
  std::vector<float> maxz(field_->W * field_->H, -1e9f);
  for (const auto& p : map_xyz_->points) {
    int gx = static_cast<int>(std::floor((p.x - field_->origin_x) / res));
    int gy = static_cast<int>(std::floor((p.y - field_->origin_y) / res));
    if (gx >= 0 && gx < field_->W && gy >= 0 && gy < field_->H) {
      int idx = gy * field_->W + gx;
      occ[idx] = 1;
      if (p.z > maxz[idx]) maxz[idx] = p.z;
    }
  }
  field_->occ = occ;
  field_->cell_maxz = maxz;

  // Compute 3x3 neighborhood fingerprint for each cell (local structure pattern).
  // This encodes the local occupancy pattern (9 bits), which changes under rotation
  // for asymmetric structures (L-shapes, corners, T-junctions). Unlike wall-direction
  // which is 0/90-symmetric, local fingerprints break yaw ambiguity.
  std::vector<uint16_t> fprint(field_->W * field_->H, 0);
  for (int y = 1; y < field_->H - 1; ++y) {
    for (int x = 1; x < field_->W - 1; ++x) {
      uint16_t fp = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          int nx = x + dx, ny = y + dy;
          int idx = ny * field_->W + nx;
          int bit = (dy + 1) * 3 + (dx + 1);  // 0..8
          if (occ[idx]) fp |= (1 << bit);
        }
      }
      fprint[y * field_->W + x] = fp;
    }
  }
  field_->cell_fprint = fprint;

  // Build per-cell mean intensity. The map points are in the SAME order as
  // map_xyz_ (both derived from map_->cloud), so we accumulate intensity per cell.
  std::vector<float> cell_inten(field_->W * field_->H, 0.0f);
  std::vector<int> cell_cnt(field_->W * field_->H, 0);
  if (!map_inten_.empty() && map_inten_.size() == map_xyz_->size()) {
    for (size_t i = 0; i < map_xyz_->size(); ++i) {
      const auto& p = map_xyz_->at(i);
      int gx = static_cast<int>(std::floor((p.x - field_->origin_x) / res));
      int gy = static_cast<int>(std::floor((p.y - field_->origin_y) / res));
      if (gx >= 0 && gx < field_->W && gy >= 0 && gy < field_->H) {
        int idx = gy * field_->W + gx;
        cell_inten[idx] += map_inten_[i];
        cell_cnt[idx] += 1;
      }
    }
    for (size_t i = 0; i < cell_inten.size(); ++i) {
      if (cell_cnt[i] > 0) cell_inten[i] /= cell_cnt[i];
    }
  }
  field_->cell_inten = cell_inten;

  // Compute gradient direction for each cell (wall orientation).
  // Sobel gradient on occupancy grid gives wall edges. Direction is the wall
  // orientation (0=horizontal, PI/2=vertical). Only meaningful where gradient
  // magnitude is high (near walls).
  std::vector<float> grad_mag(field_->W * field_->H, 0.0f);
  std::vector<float> grad_dir(field_->W * field_->H, 0.0f);
  for (int y = 1; y < field_->H - 1; ++y) {
    for (int x = 1; x < field_->W - 1; ++x) {
      // Sobel kernels
      float gx = -occ[(y-1)*field_->W + (x-1)] + occ[(y-1)*field_->W + (x+1)]
                 -2*occ[y*field_->W + (x-1)] + 2*occ[y*field_->W + (x+1)]
                 -occ[(y+1)*field_->W + (x-1)] + occ[(y+1)*field_->W + (x+1)];
      float gy = -occ[(y-1)*field_->W + (x-1)] -2*occ[(y-1)*field_->W + x] -occ[(y-1)*field_->W + (x+1)]
                 +occ[(y+1)*field_->W + (x-1)] +2*occ[(y+1)*field_->W + x] +occ[(y+1)*field_->W + (x+1)];
      int idx = y * field_->W + x;
      grad_mag[idx] = std::sqrt(gx*gx + gy*gy);
      // Direction: angle of gradient (perpendicular to wall)
      // Wall direction = gradient direction + PI/2 (mod PI)
      if (grad_mag[idx] > 0.5f) {
        float angle = std::atan2(gy, gx);
        // Normalize to [0, PI) - wall orientation is symmetric
        angle = std::fmod(angle + M_PI, M_PI);
        if (angle < 0) angle += M_PI;
        grad_dir[idx] = angle;
      }
    }
  }
  field_->cell_grad = grad_mag;
  field_->cell_dir = grad_dir;

  // Distance transform -> likelihood field (z-invariant, robust to height).
  std::vector<float> dist2 = edt2d(occ, field_->W, field_->H);
  field_->likelihood.resize(field_->W * field_->H, 0.0f);
  const float denom = 2.0f * static_cast<float>(params_.sigma * params_.sigma);
  const float inv_res2 = 1.0f / static_cast<float>(res * res);  // dist2 is in cells^2
  for (size_t i = 0; i < dist2.size(); ++i) {
    float dm2 = dist2[i] / inv_res2;  // squared meters
    field_->likelihood[i] = std::exp(-dm2 / denom);
  }

  // KdTree for the coarse z scan.
  map_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  map_tree_->setInputCloud(map_xyz_);
}

void BevMatcher::rasterizeQuery(const pcl::PointCloud<pcl::PointXYZ>& q, double yaw,
                                std::vector<std::pair<int, int>>& offsets,
                                std::vector<float>* q_maxz,
                                std::vector<uint16_t>* q_fprint,
                                std::vector<float>* q_inten,
                                const std::vector<float>* query_inten) const {
  offsets.clear();
  if (q_maxz) q_maxz->clear();
  if (q_fprint) q_fprint->clear();
  if (q_inten) q_inten->clear();
  const double res = params_.resolution;
  const double c = std::cos(yaw), s = std::sin(yaw);

  // Rasterize query points (rotated by yaw) into a temporary occupancy grid in
  // the MAP frame offset by the sensor cell. We need the 3x3 neighborhood of each
  // occupied cell, so build a local grid covering the query footprint.
  // Track per-cell: max z, intensity accumulator (mean), count.
  std::map<std::pair<int,int>, float> cell_maxz;
  std::map<std::pair<int,int>, float> cell_isum;
  std::map<std::pair<int,int>, int> cell_icnt;
  for (size_t i = 0; i < q.points.size(); ++i) {
    const auto& pt = q.points[i];
    double rx = c * pt.x - s * pt.y;
    double ry = s * pt.x + c * pt.y;
    int ox = static_cast<int>(std::lround(rx / res));
    int oy = static_cast<int>(std::lround(ry / res));
    std::pair<int,int> key(ox, oy);
    auto it = cell_maxz.find(key);
    if (it == cell_maxz.end() || pt.z > it->second) cell_maxz[key] = pt.z;
    if (query_inten && i < query_inten->size()) {
      cell_isum[key] += (*query_inten)[i];
      cell_icnt[key] += 1;
    }
  }
  if (cell_maxz.empty()) return;

  // Build a dense occupancy grid over the query footprint (with 1-cell border).
  int min_x = INT_MAX, max_x = INT_MIN, min_y = INT_MAX, max_y = INT_MIN;
  for (const auto& kv : cell_maxz) {
    min_x = std::min(min_x, kv.first.first);
    max_x = std::max(max_x, kv.first.first);
    min_y = std::min(min_y, kv.first.second);
    max_y = std::max(max_y, kv.first.second);
  }
  int gw = max_x - min_x + 3;   // +2 for border
  int gh = max_y - min_y + 3;
  std::vector<char> gocc(gw * gh, 0);
  for (const auto& kv : cell_maxz) {
    int lx = kv.first.first - min_x + 1;
    int ly = kv.first.second - min_y + 1;
    gocc[ly * gw + lx] = 1;
  }

  // Compute fingerprint per occupied cell.
  std::map<std::pair<int,int>, uint16_t> cell_fp;
  if (q_fprint) {
    for (const auto& kv : cell_maxz) {
      int lx = kv.first.first - min_x + 1;
      int ly = kv.first.second - min_y + 1;
      uint16_t fp = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          int nx = lx + dx, ny = ly + dy;
          if (nx < 0 || nx >= gw || ny < 0 || ny >= gh) continue;
          int bit = (dy + 1) * 3 + (dx + 1);
          if (gocc[ny * gw + nx]) fp |= (1 << bit);
        }
      }
      cell_fp[kv.first] = fp;
    }
  }

  offsets.reserve(cell_maxz.size());
  if (q_maxz) q_maxz->reserve(cell_maxz.size());
  if (q_fprint) q_fprint->reserve(cell_maxz.size());
  if (q_inten) q_inten->reserve(cell_maxz.size());
  for (const auto& kv : cell_maxz) {
    offsets.push_back(kv.first);
    if (q_maxz) q_maxz->push_back(kv.second);
    if (q_fprint) {
      auto it = cell_fp.find(kv.first);
      q_fprint->push_back(it != cell_fp.end() ? it->second : 0);
    }
    if (q_inten) {
      auto isum_it = cell_isum.find(kv.first);
      auto icnt_it = cell_icnt.find(kv.first);
      float mean = (isum_it != cell_isum.end() && icnt_it != cell_icnt.end() && icnt_it->second > 0)
          ? isum_it->second / icnt_it->second : 0.0f;
      q_inten->push_back(mean);
    }
  }
}

std::vector<std::pair<double, double>> BevMatcher::scanZ(
    const pcl::PointCloud<pcl::PointXYZ>& qdown, double yaw,
    double tx, double ty) const {
  const double c = std::cos(yaw), s = std::sin(yaw);
  // Rotate query into map frame at z = 0.
  std::vector<Eigen::Vector3d> qp;
  qp.reserve(qdown.size());
  for (const auto& p : qdown.points) {
    qp.emplace_back(c * p.x - s * p.y + tx, s * p.x + c * p.y + ty, p.z);
  }
  const double step = params_.z_scan_step;
  const double half = params_.z_scan_range;
  const double thresh = params_.z_inlier_dist;
  double thresh2 = thresh * thresh;

  // Collect (sensor_z, inlier_ratio) over the z scan.
  std::vector<std::pair<double, double>> curve;
  std::vector<int> idx(1);
  std::vector<float> dist2(1);
  for (double dz = -half; dz <= half; dz += step) {
    int inliers = 0;
    for (const auto& p : qp) {
      pcl::PointXYZ q;
      q.x = static_cast<float>(p.x());
      q.y = static_cast<float>(p.y());
      q.z = static_cast<float>(p.z() + dz);
      if (map_tree_->nearestKSearch(q, 1, idx, dist2) > 0 && dist2[0] <= thresh2) ++inliers;
    }
    double ratio = static_cast<double>(inliers) / static_cast<double>(qp.size());
    curve.emplace_back(dz, ratio);
  }
  // Sensor absolute z = (mean query z) + dz.
  double mean_qz = 0.0;
  for (const auto& p : qp) mean_qz += p.z();
  mean_qz = qp.empty() ? 0.0 : mean_qz / qp.size();

  // Keep the top z_keep distinct z hypotheses (by geometric overlap). Multiple
  // floors / levels can all show geometric overlap; the fine stage's
  // reflectance scoring then selects the true level.
  std::sort(curve.begin(), curve.end(),
            [](const std::pair<double,double>& a, const std::pair<double,double>& b) {
              return a.second > b.second;
            });
  std::vector<std::pair<double, double>> kept;
  for (const auto& kv : curve) {
    double z = mean_qz + kv.first;
    bool dup = false;
    for (const auto& k : kept) {
      if (std::abs(k.first - z) < params_.z_dedup) { dup = true; break; }
    }
    if (!dup) kept.emplace_back(z, kv.second);
    if (static_cast<int>(kept.size()) >= std::max(1, params_.z_keep)) break;
  }
  return kept;
}

std::vector<Candidate> BevMatcher::match(const pcl::PointCloud<pcl::PointXYZ>& query_raw) const {
  return matchImpl(query_raw, nullptr);
}

std::vector<Candidate> BevMatcher::matchWithIntensity(const pcl::PointCloud<pcl::PointXYZ>& query_raw,
                                                       const std::vector<float>& query_inten) const {
  return matchImpl(query_raw, &query_inten);
}

std::vector<Candidate> BevMatcher::matchImpl(const pcl::PointCloud<pcl::PointXYZ>& query_raw,
                                             const std::vector<float>* query_inten) const {
  ensureBuilt();
  if (query_raw.empty()) return {};
  if (!field_ || !map_tree_) throw std::runtime_error("BevMatcher: field not built");

  // Downsample the query to the BEV resolution so occupied cells are ~distinct.
  auto qdown = voxelDownXYZ(query_raw, params_.resolution);
  if (qdown->empty()) return {};

  const int W = field_->W;
  const int H = field_->H;
  const double res = params_.resolution;

  // Sparse support of the likelihood field (cells worth splatting).
  struct Sup { int idx; float val; };
  std::vector<Sup> support;
  support.reserve(field_->likelihood.size() / 4);
  for (int i = 0; i < W * H; ++i) {
    if (field_->likelihood[i] > 1e-3f) support.push_back({i, field_->likelihood[i]});
  }

  // Score buffer (mean likelihood over the query footprint per sensor cell).
  std::vector<float> score(W * H);

  struct Peak { double yaw; int tx, ty; float score; };
  std::vector<Peak> peaks;
  peaks.reserve(360 / std::max(1.0, params_.yaw_step_deg) + 4);

  std::vector<std::pair<int, int>> offsets;
  // Coarse-to-fine yaw search: first scan at 10° steps, pick top-3, then
  // refine ±5° at 1° steps. This reduces the search space and avoids local
  // minima that plague the brute-force 2° scan.
  const double yaw_coarse_step = 10.0 * M_PI / 180.0;
  const int n_yaw_coarse = std::max(1, static_cast<int>(std::ceil(2.0 * M_PI / yaw_coarse_step)));
  const double yaw_fine_step = 1.0 * M_PI / 180.0;
  const int n_yaw_fine = 11;  // ±5° at 1° = 11 candidates

  // Estimate sensor z from query median (robot is on ground).
  // This is the key simplification: z is NOT globally unknown for robot localization.
  std::vector<float> qz_vals;
  qz_vals.reserve(qdown->size());
  for (const auto& p : qdown->points) qz_vals.push_back(p.z);
  std::sort(qz_vals.begin(), qz_vals.end());
  float sensor_z_est = qz_vals.empty() ? 0.0f : qz_vals[qz_vals.size() / 2];

  for (int yi = 0; yi < n_yaw_coarse; ++yi) {
    double yaw = yaw_coarse_step * yi;
    std::vector<float> q_maxz, q_inten;
    std::vector<uint16_t> q_fp;
    rasterizeQuery(*qdown, yaw, offsets, &q_maxz, &q_fp, &q_inten, query_inten);
    if (offsets.empty()) continue;
    const double inv_n = 1.0 / static_cast<double>(offsets.size());
    bool have_fp = !q_fp.empty();
    bool have_inten = !q_inten.empty() && !field_->cell_inten.empty();

    std::fill(score.begin(), score.end(), 0.0f);
    // Cross-correlation with matching cues:
    // 1. Occupancy likelihood (distance transform) - spatial proximity
    // 2. Height consistency - query cell max-z vs map cell max-z (within z_band)
    // 3. Local structure fingerprint - 3x3 neighborhood pattern match.
    // 4. Intensity consistency - query cell mean intensity vs map cell mean.
    //    This is geometry-INDEPENDENT: different materials reflect differently,
    //    so even when occupancy/height/fingerprint all match (repetitive
    //    structure), intensity breaks the tie. It is NOT rotation-symmetric.
    const float z_band = static_cast<float>(params_.z_band);
    const float i_band = 25.0f;  // intensity tolerance (0-255 range)
    for (size_t oi = 0; oi < offsets.size(); ++oi) {
      const int ox = offsets[oi].first;
      const int oy = offsets[oi].second;
      const float qz = q_maxz[oi] + sensor_z_est;
      const uint16_t qf = have_fp ? q_fp[oi] : 0;
      const float qi = have_inten ? q_inten[oi] : 0.0f;
      for (const auto& sp : support) {
        int u = sp.idx % W;
        int v = sp.idx / W;
        int tx = u - ox;
        int ty = v - oy;
        if (tx < 0 || tx >= W || ty < 0 || ty >= H) continue;
        int midx = v * W + u;

        // Height weight (soft)
        float mz = field_->cell_maxz[midx];
        float hgap = std::abs(mz - qz);
        float hw = (hgap < z_band) ? 1.0f : std::exp(-(hgap - z_band) / z_band);

        // Fingerprint weight
        float fw = 1.0f;
        if (have_fp) {
          uint16_t mf = field_->cell_fprint[midx];
          int qbits = __builtin_popcount(qf);
          int mbits = __builtin_popcount(mf);
          if (qbits >= 3 && mbits >= 3) {
            fw = (qf == mf) ? 2.0f : 0.5f;
          }
        }

        // Intensity weight (soft): 1.0 if within band, exponential decay.
        float iw = 1.0f;
        if (have_inten) {
          float mi = field_->cell_inten[midx];
          if (mi > 0 || qi > 0) {  // skip cells with no intensity data
            float igap = std::abs(mi - qi);
            iw = (igap < i_band) ? 1.0f : std::exp(-(igap - i_band) / i_band);
          }
        }

        score[ty * W + tx] += sp.val * hw * fw * iw;
      }
    }
    // Best sensor cell for this yaw.
    float best = -1.0f;
    int bx = 0, by = 0;
    for (int ty = 0; ty < H; ++ty) {
      const float* row = &score[ty * W];
      for (int tx = 0; tx < W; ++tx) {
        if (row[tx] > best) { best = row[tx]; bx = tx; by = ty; }
      }
    }
    float mean_score = best * static_cast<float>(inv_n);
    peaks.push_back({yaw, bx, by, mean_score});
  }

  // Sort all (yaw, pos) peaks by score, then NMS into top_k diverse hypotheses.
  std::sort(peaks.begin(), peaks.end(),
            [](const Peak& a, const Peak& b) { return a.score > b.score; });

  // Take top-3 coarse peaks, refine each ±5° at 1° steps.
  int n_refine = std::min(3, static_cast<int>(peaks.size()));
  std::vector<Peak> refined;
  for (int i = 0; i < n_refine; ++i) {
    double yaw0 = peaks[i].yaw;
    for (int fi = 0; fi < n_yaw_fine; ++fi) {
      double yaw = yaw0 - 5.0 * M_PI / 180.0 + fi * yaw_fine_step;
      if (yaw < 0) yaw += 2 * M_PI;
      if (yaw >= 2 * M_PI) yaw -= 2 * M_PI;
      std::vector<float> q_maxz, q_inten;
      std::vector<uint16_t> q_fp;
      rasterizeQuery(*qdown, yaw, offsets, &q_maxz, &q_fp, &q_inten, query_inten);
      if (offsets.empty()) continue;
      const double inv_n = 1.0 / static_cast<double>(offsets.size());
      bool have_fp = !q_fp.empty();
      bool have_inten = !q_inten.empty() && !field_->cell_inten.empty();
      std::fill(score.begin(), score.end(), 0.0f);
      const float z_band = static_cast<float>(params_.z_band);
      const float i_band = 25.0f;
      for (size_t oi = 0; oi < offsets.size(); ++oi) {
        const int ox = offsets[oi].first;
        const int oy = offsets[oi].second;
        const float qz = q_maxz[oi] + sensor_z_est;
        const uint16_t qf = have_fp ? q_fp[oi] : 0;
        const float qi = have_inten ? q_inten[oi] : 0.0f;
        for (const auto& sp : support) {
          int u = sp.idx % W; int v = sp.idx / W;
          int tx = u - ox; int ty = v - oy;
          if (tx < 0 || tx >= W || ty < 0 || ty >= H) continue;
          int midx = v * W + u;
          float mz = field_->cell_maxz[midx];
          float hgap = std::abs(mz - qz);
          float hw = (hgap < z_band) ? 1.0f : std::exp(-(hgap - z_band) / z_band);
          float fw = 1.0f;
          if (have_fp) {
            uint16_t mf = field_->cell_fprint[midx];
            int qbits = __builtin_popcount(qf);
            int mbits = __builtin_popcount(mf);
            if (qbits >= 3 && mbits >= 3) fw = (qf == mf) ? 2.0f : 0.5f;
          }
          float iw = 1.0f;
          if (have_inten) {
            float mi = field_->cell_inten[midx];
            if (mi > 0 || qi > 0) {
              float igap = std::abs(mi - qi);
              iw = (igap < i_band) ? 1.0f : std::exp(-(igap - i_band) / i_band);
            }
          }
          score[ty * W + tx] += sp.val * hw * fw * iw;
        }
      }
      float best = -1.0f; int bx = 0, by = 0;
      for (int ty = 0; ty < H; ++ty) {
        const float* row = &score[ty * W];
        for (int tx = 0; tx < W; ++tx) {
          if (row[tx] > best) { best = row[tx]; bx = tx; by = ty; }
        }
      }
      float mean_score = best * static_cast<float>(inv_n);
      refined.push_back({yaw, bx, by, mean_score});
    }
  }
  // Merge refined peaks (they replace the coarse peaks for NMS).
  peaks = refined;
  std::sort(peaks.begin(), peaks.end(),
            [](const Peak& a, const Peak& b) { return a.score > b.score; });

  const double dedup_t = params_.dedup_trans / res;   // in cells
  const double dedup_r = params_.dedup_rot_deg * M_PI / 180.0;
  std::vector<Peak> kept;
  for (const auto& p : peaks) {
    if (p.score < static_cast<float>(params_.min_score)) break;
    bool dup = false;
    double px = field_->origin_x + (p.tx + 0.5) * res;
    double py = field_->origin_y + (p.ty + 0.5) * res;
    for (const auto& k : kept) {
      double kx = field_->origin_x + (k.tx + 0.5) * res;
      double ky = field_->origin_y + (k.ty + 0.5) * res;
      double dt = std::hypot(kx - px, ky - py);
      double dr = std::acos(std::clamp(std::cos(p.yaw - k.yaw), -1.0, 1.0));
      if (dt < dedup_t && dr < dedup_r) { dup = true; break; }
    }
    if (!dup) kept.push_back(p);
    if (static_cast<int>(kept.size()) >= params_.top_k) break;
  }

  // Convert peaks -> Candidates. Sensor z is fixed from the query median (robot
  // is ground-bound); no z scan needed — roll/pitch/z are constrained for robot
  // localization, only yaw + (x,y) are the real unknowns.
  std::vector<Candidate> out;
  out.reserve(kept.size());
  for (const auto& p : kept) {
    double tx = field_->origin_x + (p.tx + 0.5) * res;
    double ty = field_->origin_y + (p.ty + 0.5) * res;
    Candidate c;
    c.pose = Eigen::Isometry3d::Identity();
    c.pose.translation() = Eigen::Vector3d(tx, ty, sensor_z_est);
    c.pose.linear() = (Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    c.inlier_ratio = p.score;   // BEV score (already height-weighted)
    c.overlap = p.score;
    c.mean_residual = 0.0;
    c.refined = false;
    c.score = p.score;
    out.push_back(c);
  }
  return out;
}

}  // namespace global_reloc
