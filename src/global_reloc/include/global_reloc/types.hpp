#pragma once

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <string>
#include <vector>

namespace global_reloc {

using Isometry3d = Eigen::Isometry3d;

/// A single coarse-to-fine hypothesis.
struct Candidate {
  Isometry3d pose = Isometry3d::Identity();   ///< T_map_query, refined
  double inlier_ratio = 0.0;                   ///< fraction of query points within max_corr of map
  double tight_inlier_ratio = 0.0;             ///< fraction within tight_threshold (sharp correctness signal)
  double intensity_consistency = 0.0;          ///< reflectance match of tight inliers [0,1] (disambiguates repetitive geometry)
  double mean_residual = 0.0;                  ///< mean correspondence residual [m]
  double overlap = 0.0;                        ///< estimated map/query overlap [0,1]
  int num_inliers = 0;
  bool refined = false;                        ///< set true after fine stage
  bool has_intensity = false;                  ///< intensity_consistency is meaningful
  double score = 0.0;                           ///< final composite score [0,1]
};

/// Final relocalization output.
struct RelocResult {
  Isometry3d pose = Isometry3d::Identity();
  double score = 0.0;
  bool converged = false;
  bool gravity_aligned = false;                ///< roll/pitch consistent with gravity
  std::string failure_reason;                  ///< empty if converged
  int candidates_evaluated = 0;

  // Alignment quality of the chosen pose (post fine refinement). Useful as a
  // machine-readable correctness check: high inlier_ratio + low mean_residual
  // at the fine correspondence distance indicates a genuine alignment.
  double inlier_ratio = 0.0;     ///< fraction of query points within max_corr of map
  double tight_inlier_ratio = 0.0;  ///< fraction within tight_threshold (correctness signal)
  double intensity_consistency = 0.0;  ///< reflectance match of tight inliers [0,1]
  double mean_residual = 0.0;    ///< mean NN residual over inliers [m]
  double overlap = 0.0;
  int num_inliers = 0;
  int num_query = 0;             ///< query points evaluated for inliers

  // Reliability / ambiguity signals. Unlike inlier_ratio/mean_residual (which a
  // dense/repetitive map can game), these measure how DISTINCT the winner is:
  // a lone dominant hypothesis is trustworthy; several near-tied hypotheses mean
  // the scan is ambiguous (repetitive structure) and the pick may be wrong.
  double confidence = 0.0;       ///< distinctness of the winner vs runners-up [0,1]
  double margin = 0.0;           ///< score gap to the strongest distinct runner-up
  int num_competitors = 0;       ///< distinct runner-ups within 85% of the winner's score
};

/// Axis-aligned bounding box of the map (map frame).
struct AABB {
  Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  bool contains(const Eigen::Vector3d& p, double margin = 0.0) const {
    return (p.array() >= (min.array() - margin)).all() &&
           (p.array() <= (max.array() + margin)).all();
  }
};

// ---- parameters (POD, loaded from YAML) -----------------------------------

struct FeatureParams {
  double normal_radius = 0.6;
  double fpfh_radius = 1.2;
  int num_threads = 0;
};

struct CoarseParams {
  int num_samples = 500;
  double min_sample_distance = 0.5;
  int max_iterations = 1000;
  double max_corr_distance = 0.8;
  int top_k = 5;
  double ransac_inlier_threshold = 0.3;
};

/// Bird's-eye-view (BEV) deterministic coarse matcher parameters. Used when
/// RelocParams::coarse_strategy == "bev". This stage exploits gravity
/// alignment: roll/pitch are fixed, so only yaw + (x,y) (+ a z scan) are
/// searched globally via a distance-transform likelihood field.
struct BevParams {
  double resolution = 0.5;        ///< BEV grid cell size [m]
  double sigma = 0.35;            ///< likelihood-field Gaussian width [m]
  double yaw_step_deg = 2.0;      ///< yaw sweep resolution [deg]
  int top_k = 10;                 ///< hypotheses carried into fine stage
  double dedup_trans = 1.0;       ///< NMS translation distance [m]
  double dedup_rot_deg = 8.0;     ///< NMS angular distance [deg]
  double min_score = 0.05;        ///< drop hypotheses below this mean-likelihood
  // Coarse height (z) alignment scan: BEV is z-invariant, so z is recovered by
  // a 1D overlap scan before handing the candidate to GICP.
  double z_scan_step = 0.5;       ///< [m]
  double z_scan_range = 6.0;      ///< half-range around the centroid match [m]
  double z_inlier_dist = 1.0;     ///< NN inlier threshold for the z scan [m]
  int z_keep = 3;                 ///< distinct z hypotheses kept per BEV peak
  double z_dedup = 0.8;           ///< min separation [m] between kept z hypotheses
  double z_band = 1.0;            ///< [m] height-consistency band for BEV scoring
  double dir_band = 0.35;         ///< [rad] wall-direction consistency band (~20deg)
};

struct FineParams {
  double voxel_size = 0.2;
  double max_correspondence_distance = 1.0;
  int num_threads = 0;
  int max_iterations = 50;
  double convergence_eps = 1.0e-5;
  double tight_threshold = 0.3;   ///< sharp correctness signal: inlier dist [m]
  double intensity_tol = 0.18;    ///< reflectance match tolerance (fraction of 0..255 range)
};

struct ScoringParams {
  double min_inlier_ratio = 0.15;
  double min_tight_inlier_ratio = 0.30;   ///< sharp gate: fraction within tight_threshold
  double gravity_align_tol = 0.15;
  double map_aabb_margin = 2.0;
};

struct RelocParams {
  int accumulate_frames = 5;
  double accumulate_max_dt = 0.15;
  double voxel_query_size = 0.3;

  /// Coarse stage: "bev" (deterministic BEV likelihood-field yaw sweep,
  /// default, robust) or "sacia" (legacy FPFH + SAC-IA, fragile on sparse data).
  std::string coarse_strategy = "bev";

  FeatureParams feature;
  CoarseParams coarse;
  BevParams bev;
  FineParams fine;
  ScoringParams scoring;

  bool auto_retrigger = true;
  double reloc_cooldown = 3.0;
};

}  // namespace global_reloc
