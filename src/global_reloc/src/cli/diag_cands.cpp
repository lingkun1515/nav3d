// Diagnostic: dump every BEV->GICP candidate for one query, ranked by tight
// inlier ratio. Used to check whether the correct basin is in the candidate set
// and how it scores vs the one that gets picked.
//
// Usage: diag_cands --map <map.gkey> --query <q.pcd> [--params p.yaml] [--top 20]
#include "global_reloc/bev_matching.hpp"
#include "global_reloc/fine_matching.hpp"
#include "global_reloc/map_builder.hpp"
#include "global_reloc/params.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <iostream>
#include <string>
#include <algorithm>

using namespace global_reloc;

int main(int argc, char** argv) {
  std::string map_key, query_pcd, params_path;
  int top_override = 0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](int& k) -> std::string { return (k + 1 < argc) ? argv[++k] : ""; };
    if (a == "--map") map_key = next(i);
    else if (a == "--query") query_pcd = next(i);
    else if (a == "--params") params_path = next(i);
    else if (a == "--top") top_override = std::stoi(next(i));
  }
  if (map_key.empty() || query_pcd.empty()) {
    std::cerr << "diag_cands --map <map.gkey> --query <q.pcd> [--params p.yaml] [--top 20]\n";
    return 1;
  }

  RelocParams params;
  if (!params_path.empty()) params = loadParamsFromFile(params_path);
  if (top_override > 0) params.bev.top_k = top_override;

  auto map = std::make_shared<GlobalMap>(loadGlobalMap(map_key));
  pcl::PointCloud<pcl::PointXYZI>::Ptr qi(new pcl::PointCloud<pcl::PointXYZI>);
  bool has_i = (pcl::io::loadPCDFile<pcl::PointXYZI>(query_pcd, *qi) == 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr q(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<float> inten;
  if (has_i && !qi->empty()) {
    q->resize(qi->size()); inten.resize(qi->size());
    for (size_t i = 0; i < qi->size(); ++i) {
      q->at(i).x = qi->at(i).x; q->at(i).y = qi->at(i).y; q->at(i).z = qi->at(i).z;
      inten[i] = qi->at(i).intensity;
    }
  } else {
    pcl::io::loadPCDFile<pcl::PointXYZ>(query_pcd, *q);
  }
  std::vector<int> idx;
  pcl::removeNaNFromPointCloud(*q, *q, idx);
  if (!inten.empty() && static_cast<int>(idx.size()) == static_cast<int>(q->size())) {
    std::vector<float> clean;
    for (size_t i = 0; i < q->size(); ++i) clean.push_back(inten[idx[i]]);
    inten.swap(clean);
  }

  BevMatcher bev(params.bev);
  bev.setMap(map.get());
  std::vector<Candidate> cands = inten.empty() ? bev.match(*q) : bev.matchWithIntensity(*q, inten);
  std::cout << "BEV produced " << cands.size() << " candidates; refining each...\n";

  FineMatcher fine(params.fine);
  fine.setMap(map.get());
  const std::vector<float>* q_inten = inten.empty() ? nullptr : &inten;
  fine.refine(cands, *q, q_inten);

  // Rank by score using the SAME scorer as the production pipeline so diag
  // output reflects the real pick. Falls back to tight ratio without intensity.
  bool use_score = map->hasIntensity() && q_inten;
  if (use_score) {
    AABB aabb = map->aabb;
    // Mirror Scorer::scoreOne weights (intensity-leading).
    std::for_each(cands.begin(), cands.end(), [&](Candidate& c) {
      auto squash = [](double x, double lo, double hi){
        if (hi<=lo) return x>=hi?1.0:0.0;
        return std::clamp((x-lo)/(hi-lo),0.0,1.0);
      };
      double tight=squash(c.tight_inlier_ratio,0.1,0.6);
      double inten=squash(c.intensity_consistency,0.3,0.7);
      double resid=1.0-std::clamp(c.mean_residual/0.5,0.0,1.0);
      double inside=aabb.contains(c.pose.translation(),2.0)?1.0:0.0;
      c.score=std::clamp(0.50*inten+0.25*tight+0.08*resid+0.07*inside,0.0,1.0);
    });
    std::sort(cands.begin(), cands.end(),
              [](const Candidate& a, const Candidate& b) { return a.score > b.score; });
  } else {
    std::sort(cands.begin(), cands.end(),
              [](const Candidate& a, const Candidate& b) {
                return a.tight_inlier_ratio > b.tight_inlier_ratio;
              });
  }

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "rank  score   tight   inten   broad   resid   tx      ty      tz      yaw_deg\n";
  for (size_t i = 0; i < cands.size(); ++i) {
    const auto& c = cands[i];
    double yaw = std::atan2(c.pose.rotation()(1, 0), c.pose.rotation()(0, 0));
    Eigen::Vector3d t = c.pose.translation();
    std::cout << i << "   " << c.score << "  " << c.tight_inlier_ratio
              << "  " << c.intensity_consistency << "  " << c.inlier_ratio
              << "  " << c.mean_residual << "  "
              << t.x() << "  " << t.y() << "  " << t.z() << "  "
              << yaw * 180.0 / M_PI << "\n";
  }
  return 0;
}
