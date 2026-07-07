#include "global_reloc/coarse_matching.hpp"

#include <pcl/registration/ia_ransac.h>

#include <stdexcept>
#include <algorithm>
#include <cstdlib>

namespace global_reloc {

CoarseMatcher::CoarseMatcher(const CoarseParams& params) : params_(params) {
  if (params_.top_k <= 0) params_.top_k = 1;
  if (params_.num_samples <= 0) params_.num_samples = 3;
  if (params_.max_iterations <= 0) params_.max_iterations = 100;
}

std::vector<Candidate> CoarseMatcher::match(const FeatureCloud& query) const {
  if (!map_) throw std::runtime_error("CoarseMatcher: map not set");
  if (map_->empty() || !map_->fpfh || map_->fpfh->empty()) {
    throw std::runtime_error("CoarseMatcher: map has no features");
  }
  if (!query.fpfh || query.fpfh->empty()) {
    return {};
  }

  // SAC-IA computes its own FPFH 1-NN correspondences internally from the
  // feature clouds; we just configure it and run. Multiple rounds with
  // different RNG seeds yield distinct hypotheses for the Top-K ensemble.
  using SACIA = pcl::SampleConsensusInitialAlignment<PointXYZRN, PointXYZRN, pcl::FPFHSignature33>;

  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr qf = query.fpfh;
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr mf = map_->fpfh;

  struct Run { double fitness; Eigen::Matrix4f T; };
  std::vector<Run> runs;
  runs.reserve(static_cast<size_t>(params_.top_k));

  for (int r = 0; r < params_.top_k; ++r) {
    // Vary RNG seed per round so the ensemble isn't degenerate.
    std::srand(static_cast<unsigned>(1000 + r * 7));

    SACIA sac;
    sac.setInputSource(query.cloud);
    sac.setSourceFeatures(qf);
    sac.setInputTarget(map_->cloud);
    sac.setTargetFeatures(mf);
    sac.setMinSampleDistance(static_cast<float>(params_.min_sample_distance));
    sac.setNumberOfSamples(params_.num_samples);
    sac.setMaxCorrespondenceDistance(static_cast<float>(params_.max_corr_distance));
    sac.setMaximumIterations(params_.max_iterations);
    sac.setCorrespondenceRandomness(2 + r);  // also vary candidate pool per round

    pcl::PointCloud<PointXYZRN> aligned;
    sac.align(aligned);

    Run run;
    // getFitnessScore(threshold): mean NN sq-distance below threshold (lower=better).
    run.fitness = sac.getFitnessScore(static_cast<double>(params_.ransac_inlier_threshold));
    run.T = sac.getFinalTransformation();
    runs.push_back(run);
  }

  std::sort(runs.begin(), runs.end(),
            [](const Run& a, const Run& b) { return a.fitness < b.fitness; });

  std::vector<Candidate> out;
  const double dedup_trans = 0.5;   // [m]
  const double dedup_rot = 0.1;     // [rad]
  for (const auto& run : runs) {
    Eigen::Isometry3d iso = Eigen::Isometry3d(run.T.cast<double>());
    bool dup = false;
    for (const auto& c : out) {
      double dt = (c.pose.translation() - iso.translation()).norm();
      Eigen::Quaterniond qa(c.pose.rotation()), qb(iso.rotation());
      double dr = qa.angularDistance(qb);
      if (dt < dedup_trans && dr < dedup_rot) { dup = true; break; }
    }
    if (!dup) {
      Candidate c;
      c.pose = iso;
      // Pseudo-score from mean sq error; real inlier_ratio filled in fine/scoring.
      c.inlier_ratio = std::isfinite(run.fitness) ? 1.0 / (1.0 + run.fitness) : 0.0;
      c.mean_residual = run.fitness;
      c.refined = false;
      out.push_back(c);
      if (static_cast<int>(out.size()) >= params_.top_k) break;
    }
  }
  return out;
}

}  // namespace global_reloc
