#pragma once

#include "global_reloc/feature.hpp"
#include "global_reloc/map_builder.hpp"
#include "global_reloc/coarse_matching.hpp"
#include "global_reloc/bev_matching.hpp"
#include "global_reloc/ndt_matching.hpp"
#include "global_reloc/ndt_gicp_matcher.hpp"
#include "global_reloc/fast_global_matcher.hpp"
#include "global_reloc/fine_matching.hpp"
#include "global_reloc/scoring.hpp"
#include "global_reloc/types.hpp"

#include <memory>
#include <deque>

namespace global_reloc {

enum class RelocState { IDLE, ACCUMULATING, COARSE, FINE, DONE, FAILED };

/// Orchestrates the coarse-to-fine pipeline against a bound GlobalMap.
/// Stateless across queries except for the frame accumulation buffer.
class Relocalizer {
 public:
  Relocalizer(const RelocParams& params, std::shared_ptr<GlobalMap> map);

  /// Feed one LiDAR frame (XYZ). Returns true when enough frames accumulated
  /// to attempt relocation (accumulate_frames reached or dt exceeded).
  bool addFrame(const pcl::PointCloud<pcl::PointXYZ>& cloud, double stamp);

  /// Provide IMU-derived gravity direction in the query/LiDAR frame for the
  /// sanity check. Optional; if never called, gravity check is skipped.
  void setGravityAttitude(const Eigen::Vector3d& g_in_lidar) { gravity_ = g_in_lidar; }

  /// Provide reflectance (intensity) for the accumulated query frames, used to
  /// disambiguate repetitive geometry in scoring. Must align with the frames
  /// fed to addFrame() (concatenated in the same order). Optional.
  void setQueryIntensity(const std::vector<float>& inten) { query_intensity_ = inten; }

  /// Run the full coarse->fine->score pipeline on the accumulated frames.
  /// Clears the buffer. Returns the relocation result.
  RelocResult estimate();

  /// Direct single-shot estimate from one cloud (no accumulation). Used by CLI/tests.
  RelocResult estimateSingle(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  /// Single-shot with per-point reflectance for intensity-based disambiguation.
  RelocResult estimateSingle(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                             const std::vector<float>& intensity);

  RelocState state() const { return state_; }
  void reset();

  const RelocParams& params() const { return params_; }

 private:
  RelocParams params_;
  std::shared_ptr<GlobalMap> map_;
  FeatureExtractor feature_;
  CoarseMatcher coarse_;
  BevMatcher bev_;
  NdtMatcher ndt_;
  NdtGicpMatcher ndt_gicp_;
  FastGlobalMatcher fast_global_;
  FineMatcher fine_;
  Scorer scorer_;

  struct FrameBuf {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    double stamp = 0;
  };
  std::deque<FrameBuf> buf_;
  std::vector<float> query_intensity_;   // concatenated intensity aligned with buf_ frames
  Eigen::Vector3d gravity_{0, 0, 0};
  RelocState state_ = RelocState::IDLE;

  pcl::PointCloud<pcl::PointXYZ> mergeBuffer() const;
};

}  // namespace global_reloc
