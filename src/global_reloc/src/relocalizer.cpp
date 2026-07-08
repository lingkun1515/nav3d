#include "global_reloc/relocalizer.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <stdexcept>

namespace global_reloc {

Relocalizer::Relocalizer(const RelocParams& params, std::shared_ptr<GlobalMap> map)
    : params_(params),
      map_(std::move(map)),
      feature_(params_.feature),
      coarse_(params_.coarse),
      bev_(params_.bev),
      ndt_(params_.bev),
      ndt_gicp_(params_.bev),
      fast_global_(params_.bev),
      fine_(params_.fine),
      scorer_(params_.scoring) {
  if (!map_ || map_->empty()) {
    throw std::runtime_error("Relocalizer: map is null or empty");
  }
  coarse_.setMap(map_.get());
  bev_.setMap(map_.get());
  ndt_.setMap(map_.get());
  ndt_gicp_.setMap(map_.get());
  fast_global_.setMap(map_.get());
  fine_.setMap(map_.get());
}

void Relocalizer::reset() {
  buf_.clear();
  state_ = RelocState::IDLE;
}

bool Relocalizer::addFrame(const pcl::PointCloud<pcl::PointXYZ>& cloud, double stamp) {
  FrameBuf f;
  f.cloud = cloud;
  f.stamp = stamp;
  buf_.push_back(std::move(f));

  // Trim to accumulation window.
  while (static_cast<int>(buf_.size()) > std::max(1, params_.accumulate_frames)) {
    buf_.pop_front();
  }
  // Time-gap trigger.
  if (buf_.size() >= 2) {
    double span = buf_.back().stamp - buf_.front().stamp;
    if (span >= params_.accumulate_max_dt &&
        static_cast<int>(buf_.size()) >= std::max(1, params_.accumulate_frames)) {
      return true;
    }
  }
  return static_cast<int>(buf_.size()) >= std::max(1, params_.accumulate_frames);
}

pcl::PointCloud<pcl::PointXYZ> Relocalizer::mergeBuffer() const {
  pcl::PointCloud<pcl::PointXYZ> merged;
  for (const auto& f : buf_) merged += f.cloud;
  // Remove NaN.
  std::vector<int> idx;
  pcl::removeNaNFromPointCloud(merged, merged, idx);
  return merged;
}

RelocResult Relocalizer::estimate() {
  RelocResult res;
  if (buf_.empty()) {
    res.failure_reason = "no frames accumulated";
    state_ = RelocState::FAILED;
    return res;
  }
  state_ = RelocState::ACCUMULATING;
  pcl::PointCloud<pcl::PointXYZ> query = mergeBuffer();

  // Pre-rotate the query so it is gravity-aligned (z-up).  This fixes the
  // roll/pitch DOFs and lets the coarse matchers search only yaw+(x,y,z) —
  // effectively a roll/pitch prior from the IMU gravity direction.
  Eigen::Isometry3d pre_rot = Eigen::Isometry3d::Identity();
  if (gravity_.norm() > 0.1) {
    Eigen::Vector3d g = gravity_.normalized();
    pre_rot.linear() =
        Eigen::Quaterniond::FromTwoVectors(g, Eigen::Vector3d(0, 0, -1.0))
            .toRotationMatrix();
    pcl::transformPointCloud(query, query, pre_rot.matrix().cast<float>());
  }

  // 1. Coarse stage.
  // "ndt_gicp" = NDT (wide basin ~5-10m) 5m grid + GICP refinement (precise).
  // "ndt" = BEV pre-filter + GICP (BEV often misses correct position).
  // "bev" = BEV only (fast but yaw-ambiguous). "sacia" = FPFH + SAC-IA.
  state_ = RelocState::COARSE;
  std::vector<Candidate> cands;
  if (params_.coarse_strategy == "fast") {
    if (query.empty()) {
      res.failure_reason = "empty query";
      state_ = RelocState::FAILED;
      return res;
    }
    cands = fast_global_.match(query);
  } else if (params_.coarse_strategy == "ndt_gicp") {
    if (query.empty()) {
      res.failure_reason = "empty query";
      state_ = RelocState::FAILED;
      return res;
    }
    cands = ndt_gicp_.match(query);
  } else if (params_.coarse_strategy == "bev_ndt") {
    // BEV position pre-filter → NDT per-candidate refinement → GICP polish.
    // Faster than ndt_gicp (scans only promising cells, not full 5m grid),
    // more accurate than fast (NDT's wide basin corrects BEV position errors).
    if (query.empty()) {
      res.failure_reason = "empty query";
      state_ = RelocState::FAILED;
      return res;
    }
    auto bev_cands = bev_.match(query);
    if (!bev_cands.empty()) {
      ndt_gicp_.ensureBuilt();  // lazy-init the NDT target
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(query.makeShared());
      vg.setLeafSize(0.5f, 0.5f, 0.5f);
      auto qdown = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      vg.filter(*qdown);
      if (!qdown->empty()) {
        int n_top = std::min(10, static_cast<int>(bev_cands.size()));
        for (int i = 0; i < n_top; ++i) {
          auto ndt_r = ndt_gicp_.runNdt(*qdown, bev_cands[i].pose);
          Candidate c;
          c.pose = ndt_r.pose;
          c.score = ndt_r.fitness;       // lower fitness = better for NDT
          c.inlier_ratio = exp(-ndt_r.fitness);  // approximate inlier_ratio
          c.refined = true;
          cands.push_back(c);
        }
      }
    }
    // Fall through: if no BEV candidates or qdown empty, cands will be empty.
    // fine_.refine() will GICP-polish below (bev_ndt NOT in the skip list).
  } else if (params_.coarse_strategy == "ndt") {
    if (query.empty()) {
      res.failure_reason = "empty query";
      state_ = RelocState::FAILED;
      return res;
    }
    auto bev_cands = bev_.match(query);
    if (!bev_cands.empty()) {
      cands = ndt_.matchFromCandidates(query, bev_cands);
    } else {
      cands = ndt_.match(query);
    }
  } else if (params_.coarse_strategy == "sacia") {
    FeatureCloud fc = feature_.extract(query, params_.voxel_query_size);
    if (fc.cloud->empty()) {
      res.failure_reason = "empty query after downsample";
      state_ = RelocState::FAILED;
      return res;
    }
    cands = coarse_.match(fc);
  } else {
    if (query.empty()) {
      res.failure_reason = "empty query";
      state_ = RelocState::FAILED;
      return res;
    }
    const std::vector<float>* q_inten =
        (query_intensity_.size() == query.size()) ? &query_intensity_ : nullptr;
    if (q_inten) {
      cands = bev_.matchWithIntensity(query, *q_inten);
    } else {
      cands = bev_.match(query);
    }
  }
  if (cands.empty()) {
    res.failure_reason = "no coarse hypotheses";
    state_ = RelocState::FAILED;
    return res;
  }

  // 3. Fine refinement (skip for NDT/GICP — candidates are already GICP-refined).
  state_ = RelocState::FINE;
  if (params_.coarse_strategy != "ndt" && params_.coarse_strategy != "ndt_gicp" && params_.coarse_strategy != "fast") {
    const std::vector<float>* q_inten =
        (query_intensity_.size() == query.size()) ? &query_intensity_ : nullptr;
    fine_.refine(cands, query, q_inten);
  }

  // Undo gravity pre-rotation: T_map_orig = T_map_aligned * pre_rot.
  if (gravity_.norm() > 0.1) {
    for (auto& c : cands) {
      c.pose = c.pose * pre_rot;
    }
  }

  // 4. Score + pick.
  auto opt = scorer_.pick(std::move(cands), map_->aabb, gravity_);
  res = opt.value_or(RelocResult{});
  if (!opt.has_value()) {
    state_ = RelocState::FAILED;
  } else if (res.converged) {
    state_ = RelocState::DONE;
  } else {
    state_ = RelocState::FAILED;
  }

  buf_.clear();
  return res;
}

RelocResult Relocalizer::estimateSingle(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  query_intensity_.clear();
  buf_.clear();
  FrameBuf f;
  f.cloud = cloud;
  f.stamp = 0;
  buf_.push_back(std::move(f));
  return estimate();
}

RelocResult Relocalizer::estimateSingle(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                        const std::vector<float>& intensity) {
  query_intensity_ = intensity;
  buf_.clear();
  FrameBuf f;
  f.cloud = cloud;
  f.stamp = 0;
  buf_.push_back(std::move(f));
  return estimate();
}

}  // namespace global_reloc
