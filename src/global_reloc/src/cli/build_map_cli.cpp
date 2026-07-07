// Offline map builder CLI.
// Usage: build_map_cli --pcd <map.pcd> --out <dir> --name <map_name>
//                      [--voxel 0.5] [--normal-radius 0.6] [--fpfh-radius 1.2]
//                      [--params <params.yaml>]
#include "global_reloc/map_builder.hpp"
#include "global_reloc/params.hpp"

#include <iostream>
#include <string>

using namespace global_reloc;

int main(int argc, char** argv) {
  std::string pcd, out_dir = ".", name = "map", params_path;
  double voxel = -1, nr = -1, fr = -1;
  int nt = 0;
  bool no_intensity = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](int& k) -> std::string { return (k + 1 < argc) ? argv[++k] : ""; };
    if (a == "--pcd") pcd = next(i);
    else if (a == "--out") out_dir = next(i);
    else if (a == "--name") name = next(i);
    else if (a == "--voxel") voxel = std::stod(next(i));
    else if (a == "--normal-radius") nr = std::stod(next(i));
    else if (a == "--fpfh-radius") fr = std::stod(next(i));
    else if (a == "--threads") nt = std::stoi(next(i));
    else if (a == "--params") params_path = next(i);
    else if (a == "--no-intensity") no_intensity = true;
    else if (a == "--help" || a == "-h") {
      std::cout << "build_map_cli --pcd <map.pcd> --out <dir> --name <name> "
                   "[--voxel 0.5 --normal-radius 0.6 --fpfh-radius 1.2 "
                   "--threads 0 --params params.yaml --no-intensity]\n";
      return 0;
    }
  }
  if (pcd.empty()) {
    std::cerr << "error: --pcd required\n";
    return 1;
  }

  MapBuilder::BuildOptions opts;
  opts.keep_intensity = !no_intensity;
  if (!params_path.empty()) {
    RelocParams p = loadParamsFromFile(params_path);
    opts.feature = p.feature;
    if (voxel < 0) opts.voxel_size = 0.5;  // not in yaml; keep default
  }
  if (voxel > 0) opts.voxel_size = voxel;
  if (nr > 0) opts.feature.normal_radius = nr;
  if (fr > 0) opts.feature.fpfh_radius = fr;
  opts.feature.num_threads = nt;

  MapBuilder builder(opts);
  try {
    GlobalMap m = builder.buildFromPCD(pcd);
    std::string gkey = saveGlobalMap(m, out_dir, name);
    std::cout << "map built: " << gkey << "  (" << m.size() << " pts)\n";
  } catch (const std::exception& e) {
    std::cerr << "build failed: " << e.what() << "\n";
    return 2;
  }
  return 0;
}
