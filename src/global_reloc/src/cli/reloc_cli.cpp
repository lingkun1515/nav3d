// Offline reloc CLI (no ROS). Loads a .gkey map and a query PCD, runs the
// coarse-to-fine pipeline, prints the estimated pose + score.
// Usage: reloc_cli --map <map.gkey> --query <cloud.pcd> [--params params.yaml]
#include "global_reloc/relocalizer.hpp"
#include "global_reloc/map_builder.hpp"
#include "global_reloc/params.hpp"
#include "global_reloc/ndt_matching.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include <iostream>
#include <string>
#include <iomanip>

using namespace global_reloc;

int main(int argc, char** argv) {
  std::string map_key, query_pcd, params_path, init_pose_str;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](int& k) -> std::string { return (k + 1 < argc) ? argv[++k] : ""; };
    if (a == "--map") map_key = next(i);
    else if (a == "--query") query_pcd = next(i);
    else if (a == "--params") params_path = next(i);
    else if (a == "--init-pose") init_pose_str = next(i);
    else if (a == "--help" || a == "-h") {
      std::cout << "reloc_cli --map <map.gkey> --query <cloud.pcd> [--params p.yaml]\n"
                << "        [--init-pose \"tx ty tz qx qy qz qw\"]  (tracking mode: local GICP from prior)\n";
      return 0;
    }
  }
  if (map_key.empty() || query_pcd.empty()) {
    std::cerr << "error: --map and --query required\n";
    return 1;
  }

  RelocParams params;
  if (!params_path.empty()) {
    try { params = loadParamsFromFile(params_path); }
    catch (const std::exception& e) { std::cerr << "params load: " << e.what() << "\n"; }
  }

  auto map = std::make_shared<GlobalMap>(loadGlobalMap(map_key));
  std::cout << "map loaded: " << map->size() << " pts"
            << (map->hasIntensity() ? " (with intensity)" : "") << "\n";

  // Load query as XYZI so reflectance can disambiguate repetitive geometry.
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
  } else if (pcl::io::loadPCDFile<pcl::PointXYZ>(query_pcd, *q) < 0) {
    std::cerr << "failed to load query: " << query_pcd << "\n";
    return 2;
  }
  std::vector<int> idx;
  pcl::removeNaNFromPointCloud(*q, *q, idx);
  if (!inten.empty()) {
    // removeNaNFromPointCloud drops NaN xyz points; rebuild intensity to match.
    std::vector<float> clean;
    clean.reserve(q->size());
    for (size_t i = 0; i < q->size(); ++i) clean.push_back(inten[idx[i]]);
    inten.swap(clean);
  }

  // If --init-pose given, use NDTMatcher directly with a single local GICP
  // from the prior (tracking mode). Otherwise full global search.
  RelocResult r;
  if (!init_pose_str.empty()) {
    // Parse "tx ty tz qx qy qz qw"
    std::stringstream ss(init_pose_str);
    double tx, ty, tz, qx, qy, qz, qw;
    ss >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
    init.translation() = Eigen::Vector3d(tx, ty, tz);
    init.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();

    // Use NDT matcher with a single GICP from the prior pose.
    NdtMatcher ndt(params.bev);
    ndt.setMap(map.get());
    // Downsample query for speed.
    pcl::PointCloud<pcl::PointXYZ>::Ptr qdown(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(q->makeShared());
    vg.setLeafSize(params.bev.resolution, params.bev.resolution, params.bev.resolution);
    vg.filter(*qdown);
    std::vector<Eigen::Vector3d> q_pts(qdown->size());
    for (size_t i = 0; i < qdown->size(); ++i)
      q_pts[i] = qdown->at(i).getVector3fMap().cast<double>();

    // Build small_gicp map + tree once.
    pcl::PointCloud<pcl::PointXYZ>::Ptr mxyz(new pcl::PointCloud<pcl::PointXYZ>);
    mxyz->resize(map->cloud->size());
    for (size_t i = 0; i < map->cloud->size(); ++i) {
      mxyz->at(i).x = map->cloud->at(i).x;
      mxyz->at(i).y = map->cloud->at(i).y;
      mxyz->at(i).z = map->cloud->at(i).z;
    }
    std::vector<Eigen::Vector3d> m_pts(mxyz->size());
    for (size_t i = 0; i < mxyz->size(); ++i)
      m_pts[i] = mxyz->at(i).getVector3fMap().cast<double>();
    // preprocess_points with downsampling_resolution: a POSITIVE value enables
    // voxel downsampling; 0.0 is NOT "no downsampling" — small_gicp treats 0 as
    // a degenerate voxel size and collapses the cloud to ~1 point. Pass a small
    // positive resolution (map already 0.4m-downsampled; 0.1 keeps detail).
    auto [mpc, mtree] = small_gicp::preprocess_points(m_pts, 0.1, 10, 1);
    auto [qpc, qtree] = small_gicp::preprocess_points(q_pts, 0.1, 10, 1);
    (void)qtree;

    small_gicp::RegistrationSetting s;
    s.type = small_gicp::RegistrationSetting::GICP;
    s.max_correspondence_distance = 5.0;  // match NdtMatcher's wide basin
    s.max_iterations = 100;
    s.num_threads = 1;
    s.rotation_eps = 0.5 * M_PI / 180.0;
    s.translation_eps = 1e-3;
    small_gicp::RegistrationResult rr =
        small_gicp::align(*mpc, *qpc, *mtree, init, s);

    // Evaluate overlap with PCL KdTree.
    pcl::KdTreeFLANN<pcl::PointXYZ> mtree_pcl;
    mtree_pcl.setInputCloud(mxyz);
    std::vector<int> nn_idx(1); std::vector<float> nn_dist2(1);
    int inliers = 0, tight = 0, n = 0;
    double sum = 0;
    for (const auto& p : q_pts) {
      pcl::PointXYZ q2;
      Eigen::Vector3d tp = rr.T_target_source * p;
      q2.x = static_cast<float>(tp.x()); q2.y = static_cast<float>(tp.y()); q2.z = static_cast<float>(tp.z());
      if (mtree_pcl.nearestKSearch(q2, 1, nn_idx, nn_dist2) > 0) {
        ++n;
        if (nn_dist2[0] <= 2.25) { ++inliers; sum += std::sqrt(nn_dist2[0]); if (nn_dist2[0] <= 0.09) ++tight; }
      }
    }
    r.pose = rr.T_target_source;
    r.score = n ? static_cast<double>(tight) / n : 0;
    r.converged = rr.converged;
    r.inlier_ratio = n ? static_cast<double>(inliers) / n : 0;
    r.tight_inlier_ratio = n ? static_cast<double>(tight) / n : 0;
    r.mean_residual = inliers > 0 ? sum / inliers : 2.0;
    r.candidates_evaluated = 1;
  } else {
    Relocalizer reloc(params, map);
    r = inten.empty() ? reloc.estimateSingle(*q) : reloc.estimateSingle(*q, inten);
  }
  r.num_query = static_cast<int>(q->size());

  std::cout << std::fixed << std::setprecision(4);
  if (r.converged) {
    std::cout << "RELOC OK  score=" << r.score
              << " cands=" << r.candidates_evaluated << "\n";
  } else {
    std::cout << "RELOC FAIL reason=" << r.failure_reason
              << " score=" << r.score << "\n";
  }
  Eigen::Quaterniond q_(r.pose.rotation());
  Eigen::Vector3d t = r.pose.translation();
  std::cout << "T = [tx ty tz qx qy qz qw] "
            << t.x() << " " << t.y() << " " << t.z() << " "
            << q_.x() << " " << q_.y() << " " << q_.z() << " " << q_.w() << "\n";
  // Machine-readable alignment-quality metrics. tight_inlier_ratio (fraction
  // within tight_threshold) is the sharp correctness signal; mean_residual is
  // secondary. A genuine alignment has high tight_inlier_ratio; a wrong basin
  // in a dense/repetitive map does not.
  std::cout << "ALIGN inlier_ratio=" << r.inlier_ratio
            << " tight_inlier_ratio=" << r.tight_inlier_ratio
            << " intensity_consistency=" << r.intensity_consistency
            << " mean_residual=" << r.mean_residual
            << " num_inliers=" << r.num_inliers
            << " num_query=" << r.num_query << "\n";
  std::cout << "CONF confidence=" << r.confidence
            << " margin=" << r.margin
            << " competitors=" << r.num_competitors << "\n";
  return r.converged ? 0 : 3;
}
