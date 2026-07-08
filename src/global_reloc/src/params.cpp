#include "global_reloc/params.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace global_reloc {

namespace {

std::string trim(const std::string& s) {
  size_t a = s.find_first_not_of(" \t\r\n");
  size_t b = s.find_last_not_of(" \t\r\n");
  return (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
}

bool parseScalar(const std::string& v, double& out) {
  try {
    size_t pos = 0;
    double d = std::stod(v, &pos);
    if (pos != v.size()) return false;
    out = d;
    return true;
  } catch (...) { return false; }
}
bool parseScalar(const std::string& v, int& out) {
  try {
    size_t pos = 0;
    int i = std::stoi(v, &pos);
    if (pos != v.size()) return false;
    out = i;
    return true;
  } catch (...) { return false; }
}
bool parseScalar(const std::string& v, bool& out) {
  std::string t = v;
  std::transform(t.begin(), t.end(), t.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (t == "true" || t == "yes" || t == "1") { out = true; return true; }
  if (t == "false" || t == "no" || t == "0") { out = false; return true; }
  return false;
}

// A flat lookup: section.key -> raw string value. We only handle our own
// params.yaml shape (2-space nested groups). Top-level (no prefix) entries
// like `accumulate_frames` live under the "" prefix.
struct FlatMap {
  std::unordered_map<std::string, std::string> kv;
  bool get(const std::string& key, std::string& out) const {
    auto it = kv.find(key);
    if (it == kv.end()) return false;
    out = it->second;
    return true;
  }
  template <class T>
  bool getAs(const std::string& key, T& out) const {
    std::string v;
    if (!get(key, v)) return false;
    T tmp;
    if (!parseScalar(v, tmp)) return false;
    out = tmp;
    return true;
  }
};

FlatMap parseFlat(const std::string& yaml) {
  // Tracks current section path by indentation depth.
  // depth 0 => root prefix "" (entries under global_reloc: header are depth 1 => "")
  // A header line "key:" with no value opens a section.
  std::vector<std::pair<int, std::string>> stack;  // (indent, name)
  FlatMap m;
  std::istringstream is(yaml);
  std::string line;
  while (std::getline(is, line)) {
    // strip comments
    size_t h = line.find('#');
    if (h != std::string::npos) line = line.substr(0, h);
    std::string t = trim(line);
    if (t.empty()) continue;

    // count leading spaces (2 per level assumed)
    size_t indent = 0;
    while (indent < line.size() && (line[indent] == ' ')) ++indent;
    std::string content = trim(line);

    // pop stack to current indent
    while (!stack.empty() && stack.back().first >= indent) stack.pop_back();

    size_t colon = content.find(':');
    if (colon == std::string::npos) continue;
    std::string key = trim(content.substr(0, colon));
    std::string val = trim(content.substr(colon + 1));

    // build dotted prefix from stack (skip the top-level "global_reloc:" header)
    std::string prefix;
    for (auto& [d, n] : stack) {
      if (!prefix.empty()) prefix += ".";
      prefix += n;
    }
    std::string full = prefix.empty() ? key : (prefix + "." + key);

    if (val.empty()) {
      // section header — push with its indent (so children > this indent)
      stack.emplace_back(static_cast<int>(indent), key);
    } else {
      // strip optional surrounding quotes
      if (val.size() >= 2 && ((val.front() == '"' && val.back() == '"') ||
                               (val.front() == '\'' && val.back() == '\''))) {
        val = val.substr(1, val.size() - 2);
      }
      m.kv[full] = val;
    }
  }
  return m;
}

void apply(const FlatMap& m, RelocParams& p) {
  // strip "global_reloc." prefix used in params.yaml
  auto P = [&](const std::string& k) {
    return "global_reloc." + k;
  };
  m.getAs(P("accumulate_frames"), p.accumulate_frames);
  m.getAs(P("accumulate_max_dt"), p.accumulate_max_dt);
  m.getAs(P("voxel_query_size"), p.voxel_query_size);

  {
    std::string s;
    if (m.get(P("coarse_strategy"), s)) {
      // strip quotes if any
      if (s.size() >= 2 && s.front() == '"' && s.back() == '"') s = s.substr(1, s.size() - 2);
      if (!s.empty()) p.coarse_strategy = s;
    }
  }

  m.getAs(P("feature.normal_radius"), p.feature.normal_radius);
  m.getAs(P("feature.fpfh_radius"), p.feature.fpfh_radius);
  m.getAs(P("feature.num_threads"), p.feature.num_threads);

  m.getAs(P("coarse.num_samples"), p.coarse.num_samples);
  m.getAs(P("coarse.min_sample_distance"), p.coarse.min_sample_distance);
  m.getAs(P("coarse.max_iterations"), p.coarse.max_iterations);
  m.getAs(P("coarse.max_corr_distance"), p.coarse.max_corr_distance);
  m.getAs(P("coarse.top_k"), p.coarse.top_k);
  m.getAs(P("coarse.ransac_inlier_threshold"), p.coarse.ransac_inlier_threshold);

  m.getAs(P("bev.resolution"), p.bev.resolution);
  m.getAs(P("bev.sigma"), p.bev.sigma);
  m.getAs(P("bev.yaw_step_deg"), p.bev.yaw_step_deg);
  m.getAs(P("bev.top_k"), p.bev.top_k);
  m.getAs(P("bev.dedup_trans"), p.bev.dedup_trans);
  m.getAs(P("bev.dedup_rot_deg"), p.bev.dedup_rot_deg);
  m.getAs(P("bev.min_score"), p.bev.min_score);
  m.getAs(P("bev.z_scan_step"), p.bev.z_scan_step);
  m.getAs(P("bev.z_scan_range"), p.bev.z_scan_range);
  m.getAs(P("bev.z_inlier_dist"), p.bev.z_inlier_dist);
  m.getAs(P("bev.z_keep"), p.bev.z_keep);
  m.getAs(P("bev.z_dedup"), p.bev.z_dedup);
  m.getAs(P("bev.z_band"), p.bev.z_band);
  m.getAs(P("bev.dir_band"), p.bev.dir_band);
  m.getAs(P("bev.ndt_grid_step"), p.bev.ndt_grid_step);
  m.getAs(P("bev.ndt_yaw_count"), p.bev.ndt_yaw_count);

  m.getAs(P("fine.voxel_size"), p.fine.voxel_size);
  m.getAs(P("fine.max_correspondence_distance"), p.fine.max_correspondence_distance);
  m.getAs(P("fine.num_threads"), p.fine.num_threads);
  m.getAs(P("fine.max_iterations"), p.fine.max_iterations);
  m.getAs(P("fine.convergence_eps"), p.fine.convergence_eps);
  m.getAs(P("fine.tight_threshold"), p.fine.tight_threshold);
  m.getAs(P("fine.intensity_tol"), p.fine.intensity_tol);

  m.getAs(P("scoring.min_inlier_ratio"), p.scoring.min_inlier_ratio);
  m.getAs(P("scoring.min_tight_inlier_ratio"), p.scoring.min_tight_inlier_ratio);
  m.getAs(P("scoring.gravity_align_tol"), p.scoring.gravity_align_tol);
  m.getAs(P("scoring.map_aabb_margin"), p.scoring.map_aabb_margin);

  m.getAs(P("runtime.auto_retrigger"), p.auto_retrigger);
  m.getAs(P("runtime.reloc_cooldown"), p.reloc_cooldown);

  // Node-level params (shared file, used by reloc_node ROS wrapper).
  m.getAs(P("gate_publish"), p.gate_publish);
  m.getAs(P("gravity_pitch_deg"), p.gravity_pitch_deg);
}

}  // namespace

RelocParams loadParamsFromString(const std::string& yaml) {
  RelocParams p;
  apply(parseFlat(yaml), p);
  return p;
}

RelocParams loadParamsFromFile(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open params file: " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return loadParamsFromString(ss.str());
}

}  // namespace global_reloc
