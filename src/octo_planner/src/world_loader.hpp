#ifndef WORLD_LOADER_HPP_
#define WORLD_LOADER_HPP_

#include <memory>
#include <string>

namespace octomap {
class OcTree;
}

std::shared_ptr<octomap::OcTree> loadWorldToOctomap(
    const std::string & world_file,
    double resolution,
    double xy_window_size_m = 24.0,
    double ground_surface_max_thickness_m = 0.6,
    bool enable_stair_step_surface_mode = true,
    double stair_step_max_height_m = 0.5,
    double stair_step_max_depth_m = 0.8,
    double stair_step_min_width_m = 1.0);

#endif  // WORLD_LOADER_HPP_
