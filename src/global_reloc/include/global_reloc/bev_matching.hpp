#pragma once

#include "global_reloc/types.hpp"
#include "global_reloc/map_builder.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <memory>
#include <vector>

namespace global_reloc {

/// Deterministic coarse matcher exploiting gravity alignment.
///
/// Since map and query are gravity aligned, roll/pitch are fixed and the only
/// unknowns are yaw + (x,y) translation (plus a z offset recovered by a 1D
/// scan). Instead of fragile feature correspondence (FPFH + SAC-IA), this
/// matcher rasterizes both clouds to a 2D bird's-eye-view occupancy grid,
/// builds a distance-transform likelihood field of the map, and brute-forces
/// yaw while recovering the best in-plane translation by correlating the query
/// footprint against the map likelihood field. It is deterministic and global —
/// no initial guess, no randomness.
class BevMatcher {
 public:
  explicit BevMatcher(const BevParams& params);

  /// Bind the pre-built global map. Lazily builds the BEV likelihood field
  /// and a KdTree (for the z scan + overlap) on first use.
  void setMap(const GlobalMap* map);

  /// Run deterministic global coarse matching on a raw XYZ query cloud.
  /// Returns up to `top_k` Candidates (yaw set, z set, refined=false).
  std::vector<Candidate> match(const pcl::PointCloud<pcl::PointXYZ>& query_raw) const;

  /// Variant that also passes per-point query intensity for the intensity
  /// disambiguation channel (used when the map carries reflectance).
  std::vector<Candidate> matchWithIntensity(const pcl::PointCloud<pcl::PointXYZ>& query_raw,
                                            const std::vector<float>& query_inten) const;

 private:
  std::vector<Candidate> matchImpl(const pcl::PointCloud<pcl::PointXYZ>& query_raw,
                                   const std::vector<float>* query_inten) const;
  BevParams params_;
  const GlobalMap* map_ = nullptr;
  std::vector<float> map_inten_;  ///< map intensity (parallel to map_xyz_)

  struct Field {
    double resolution = 0.5;
    int W = 0, H = 0;
    double origin_x = 0.0, origin_y = 0.0;   ///< map-corner XY of cell (0,0) center
    std::vector<float> likelihood;          ///< exp(-dist^2 / 2sigma^2), row-major y*W+x
    // Height channel: for each occupied cell, store the dominant (highest) z so
    // the coarse stage can discriminate look-alike structures at different
    // heights (e.g. ceiling vs floor) — critical when the BEV occupancy alone
    // is ambiguous. Per-cell max-z mirrors the map's 3D structure into the 2D field.
    std::vector<float> cell_maxz;           ///< valid only where occupancy != 0
    std::vector<char> occ;                  ///< occupancy mask (1 = map point present)
    // Direction channel: per-cell wall orientation (0-PI radians) from Sobel
    // gradient of the occupancy grid. A wall running along X has dir~0, along Y
    // has dir~PI/2. Matching query wall direction (rotated by yaw) against the
    // map cell direction is the key yaw discriminator — pure occupancy is
    // rotation-symmetric in structured environments.
    std::vector<float> cell_dir;            ///< 0..PI, valid where occ && |grad|>0
    std::vector<float> cell_grad;           ///< gradient magnitude (confidence of dir)
    // Local structure fingerprint: for each occupied cell, a compact encoding of
    // its 3x3 neighborhood occupancy (9 bits). This is rotation-INVARIANT only
    // for symmetric patterns; for asymmetric local structure (L-shapes, corners),
    // the fingerprint changes under rotation -> strong yaw discriminator, unlike
    // wall-direction which is 0/90-symmetric. Stored as uint16 [0..511].
    std::vector<uint16_t> cell_fprint;      ///< 3x3 neighborhood occupancy pattern
    // Intensity channel: mean reflectance per cell. The map carries reflectance
    // (0-255); matching query intensity against map cell intensity is a powerful
    // disambiguator that pure geometry lacks — different materials (wall vs glass
    // vs metal) have distinct reflectance even at identical geometry.
    std::vector<float> cell_inten;          ///< mean intensity per occupied cell
  };
  std::shared_ptr<Field> field_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_xyz_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_tree_;

  void buildField();
  void ensureBuilt() const;

  /// Rasterize rotated query points to distinct occupied cell offsets (in cells,
  /// relative to the sensor origin). q_maxz/q_fprint/q_inten (optional) return
  /// the max query z / 3x3-neighborhood fingerprint / mean intensity per cell.
  void rasterizeQuery(const pcl::PointCloud<pcl::PointXYZ>& q, double yaw,
                      std::vector<std::pair<int, int>>& offsets,
                      std::vector<float>* q_maxz = nullptr,
                      std::vector<uint16_t>* q_fprint = nullptr,
                      std::vector<float>* q_inten = nullptr,
                      const std::vector<float>* query_inten = nullptr) const;

  /// Coarse 1D z scan: given in-plane (x,y,yaw), find the top z_keep distinct
  /// z levels by query/map NN overlap. Returns (sensor_z, inlier_ratio) pairs.
  std::vector<std::pair<double, double>> scanZ(const pcl::PointCloud<pcl::PointXYZ>& qdown,
                                               double yaw, double tx, double ty) const;
};

}  // namespace global_reloc
