#include "global_reloc/scoring.hpp"

#include <algorithm>
#include <cmath>

namespace global_reloc {

namespace {
// Logistic squash into [0,1]; 0 at x<=lo, ~1 at x>=hi.
double squash(double x, double lo, double hi) {
  if (hi <= lo) return x >= hi ? 1.0 : 0.0;
  double t = (x - lo) / (hi - lo);
  t = std::clamp(t, 0.0, 1.0);
  return t;
}
}  // namespace

Scorer::Scorer(const ScoringParams& params) : params_(params) {}

void Scorer::scoreOne(Candidate& c, const AABB& map_aabb) const {
  // Primary discriminant is the TIGHT inlier ratio (fraction within the sharp
  // tight_threshold): a genuine alignment has most points tight, while a wrong
  // basin in a dense/repetitive map has broad but few tight matches. When
  // reflectance (intensity) consistency is available, it is the strongest
  // tiebreaker — repetitive geometric structures differ in material/reflectance,
  // so the true location matches on intensity while look-alikes do not.
  double tight = squash(c.tight_inlier_ratio, 0.1, 0.6);
  double overlap = squash(c.overlap, 0.1, 0.5);
  double resid = 1.0 - std::clamp(c.mean_residual / 0.5, 0.0, 1.0);
  double inside = map_aabb.contains(c.pose.translation(), params_.map_aabb_margin) ? 1.0 : 0.0;

  double s;
  if (c.has_intensity) {
    // Reflectance is the LEADING term (strongest disambiguator); geometry
    // (tight inliers) is secondary. We weight intensity highest because in this
    // dense/repetitive indoor map, geometry alone cannot break yaw/position
    // ambiguity but material reflectance can.
    double inten = squash(c.intensity_consistency, 0.3, 0.7);
    s = 0.50 * inten + 0.25 * tight + 0.10 * overlap + 0.08 * resid + 0.07 * inside;
  } else {
    s = 0.55 * tight + 0.20 * overlap + 0.15 * resid + 0.10 * inside;
  }
  c.score = std::clamp(s, 0.0, 1.0);
}

std::optional<RelocResult> Scorer::pick(
    std::vector<Candidate> candidates,
    const AABB& map_aabb,
    const Eigen::Vector3d& gravity_attitude) const {
  RelocResult res;
  res.candidates_evaluated = static_cast<int>(candidates.size());

  if (candidates.empty()) {
    res.failure_reason = "no coarse candidates";
    return res;
  }

  // Score all, sort by score desc.
  for (auto& c : candidates) scoreOne(c, map_aabb);
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.score > b.score; });

  // Reliability: how distinct is the winner? Compare the top hypothesis to the
  // strongest DISTINCT runner-up (translation > distinct_thresh, i.e. a truly
  // different place). A wide gap => the scan matched uniquely; a near-tie =>
  // repetitive structure, the pick is ambiguous and may be wrong. This is
  // independent of the (gameable) inlier/residual metrics.
  const Candidate& winner = candidates.front();
  const double distinct_thresh = 1.5;   // [m] below this two hypotheses are the same place
  double runner_score = 0.0;
  int num_competitors = 0;
  for (size_t i = 1; i < candidates.size(); ++i) {
    double dt = (candidates[i].pose.translation() - winner.pose.translation()).norm();
    if (dt < distinct_thresh) continue;            // same basin as the winner
    if (candidates[i].score >= 0.85 * winner.score) ++num_competitors;
    if (candidates[i].score > runner_score) runner_score = candidates[i].score;
  }
  double margin = std::max(0.0, winner.score - runner_score);
  // confidence: 1 when the winner has no distinct rival, falling off as the
  // runner-up closes the gap (margin normalized ~0.15) and as rivals multiply.
  double conf = (num_competitors == 0 && runner_score == 0.0)
      ? 1.0
      : std::clamp(margin / 0.15, 0.0, 1.0) / (1.0 + num_competitors);
  res.margin = margin;
  res.num_competitors = num_competitors;
  res.confidence = std::clamp(conf, 0.0, 1.0);

  // First candidate passing sanity filters.
  const Candidate* best = nullptr;
  for (const auto& c : candidates) {
    if (c.inlier_ratio < params_.min_inlier_ratio) continue;
    if (c.tight_inlier_ratio < params_.min_tight_inlier_ratio) continue;
    if (!c.refined) continue;
    if (!map_aabb.contains(c.pose.translation(), params_.map_aabb_margin)) continue;
    // Gravity: if provided, the query frame's gravity direction (rotated into
    // map frame by c.pose) must be close to map +Z (gravity down == -Z, so the
    // sensor's up should remain ~+Z when roll/pitch correct).
    if (gravityValid(gravity_attitude)) {
      Eigen::Vector3d g_map = c.pose.rotation() * gravity_attitude.normalized();
      double dev = std::acos(std::clamp(g_map.z(), -1.0, 1.0));
      if (dev > params_.gravity_align_tol) continue;
    }
    best = &c;
    break;
  }

  if (!best) {
    // Fall back to highest-score even if it failed a filter, but flag not converged.
    const Candidate& top = candidates.front();
    if (top.tight_inlier_ratio < params_.min_tight_inlier_ratio &&
        top.inlier_ratio < params_.min_inlier_ratio) {
      res.failure_reason = "low inlier ratio (broad=" + std::to_string(top.inlier_ratio) +
                           " tight=" + std::to_string(top.tight_inlier_ratio) + ")";
      return res;
    }
    res.pose = top.pose;
    res.score = top.score;
    res.converged = false;
    res.inlier_ratio = top.inlier_ratio;
    res.tight_inlier_ratio = top.tight_inlier_ratio;
    res.intensity_consistency = top.has_intensity ? top.intensity_consistency : 0.0;
    res.mean_residual = top.mean_residual;
    res.overlap = top.overlap;
    res.num_inliers = top.num_inliers;
    res.failure_reason = "passed score but failed a sanity filter (tight=" +
                         std::to_string(top.tight_inlier_ratio) + ")";
    return res;
  }

  res.pose = best->pose;
  res.score = best->score;
  res.converged = true;
  res.gravity_aligned = gravityValid(gravity_attitude);
  res.inlier_ratio = best->inlier_ratio;
  res.tight_inlier_ratio = best->tight_inlier_ratio;
  res.intensity_consistency = best->has_intensity ? best->intensity_consistency : 0.0;
  res.mean_residual = best->mean_residual;
  res.overlap = best->overlap;
  res.num_inliers = best->num_inliers;
  return res;
}

}  // namespace global_reloc
