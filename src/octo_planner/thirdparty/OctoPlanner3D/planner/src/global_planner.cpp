#include "global_planner.h"

#include <algorithm>
#include <climits>
#include <map>
#include <utility>
#include <chrono>

namespace global_planner
{

    GlobalPlanner::GlobalPlanner():map_ready_(false),has_start_(false),has_goal_(false),planning_in_progress_(false),plan_seq_(0),last_success_seq_(0)
    {
        printf("GlobalPlanner Constructure!!! \n");
    }

    GlobalPlanner::~GlobalPlanner()
    {
        printf("GlobalPlanner Destructure!!! \n");
    }

    void GlobalPlanner::configure(const PlannerConfig& config)
    {
        robot_radius_ = config.robot_radius;
        max_iterations_ = config.max_iterations;
        snap_search_radius_cells_ = config.snap_search_radius_cells;
        require_ground_support_ = config.require_ground_support;
        strict_direct_ground_support_ = config.strict_direct_ground_support;
        ground_support_xy_radius_cells_ = config.ground_support_xy_radius_cells;
        ground_support_depth_cells_ = config.ground_support_depth_cells;
        enable_preblocked_costmap_ = config.enable_preblocked_costmap;
        preblocked_costmap_radius_cells_ = config.preblocked_costmap_radius_cells;
        preblocked_costmap_weight_ = config.preblocked_costmap_weight;
        lowest_traversable_only_ = config.lowest_traversable_only;
        radical_infill_enabled_ = config.radical_infill_enabled;
        radical_infill_radius_m_ = config.radical_infill_radius_m;
        radical_infill_clearance_m_ = config.radical_infill_clearance_m;
        radical_infill_half_height_m_ = config.radical_infill_half_height_m;
        preblocked_hard_obstacle_ = config.preblocked_hard_obstacle;
        flatten_enabled_ = config.flatten_enabled;
        flatten_window_cells_ = config.flatten_window_cells;
        flatten_max_delta_cells_ = config.flatten_max_delta_cells;
    }

    const std::unordered_set<GridIndex, GridIndexHash>& GlobalPlanner::getTraversableCells() const
    {
        return traversable_cells_;
    }

    const std::unordered_set<GridIndex, GridIndexHash>& GlobalPlanner::getPreblockedCells() const
    {
        return preblocked_cells_;
    }

    const std::unordered_map<GridIndex, double, GridIndexHash>& GlobalPlanner::getPreblockedCostmap() const
    {
        return preblocked_costmap_;
    }

    octomap::point3d GlobalPlanner::gridToWorldPublic(const GridIndex& idx) const
    {
        return gridToWorld(idx);
    }

    double GlobalPlanner::getResolution() const
    {
        if (!octree_) return 0.2;
        return octree_->getResolution();
    }

    void GlobalPlanner::setOctomap(std::shared_ptr<octomap::OcTree> map)
    {
        if (!map)
        {
            printf("Octomap is Null!!! return.\n");
            return;
        }

        if (octree_ == map)
        {
            printf("Octomap is No Update!!! return.\n");
            return;
        }

        octree_ = map;
        map_ready_ = true;

        // Cache metric bounds once — isInsideMetricBounds() is called millions of times
        octree_->getMetricMin(metric_min_x_, metric_min_y_, metric_min_z_);
        octree_->getMetricMax(metric_max_x_, metric_max_y_, metric_max_z_);
        cached_resolution_ = octree_->getResolution();

        auto t0 = std::chrono::steady_clock::now();
        // Build occupied snapshot (cached for A*, avoids octree access during search)
        occupied_set_.clear();
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (octree_->isNodeOccupied(*it))
               occupied_set_.insert(worldToGrid(it.getX(), it.getY(), it.getZ()));
       }
       auto t1 = std::chrono::steady_clock::now();
        initGridLookup();
       printf("[setOctomap] occupied_set: %zu (%.1f ms)\n", occupied_set_.size(),
              std::chrono::duration<double, std::milli>(t1 - t0).count());

        rebuildPreblockedCells();
        auto t2 = std::chrono::steady_clock::now();
        printf("[setOctomap] rebuildPreblockedCells: %zu (%.1f ms)\n", preblocked_cells_.size(),
               std::chrono::duration<double, std::milli>(t2 - t1).count());

        rebuildDerivedLayers();
        auto t3 = std::chrono::steady_clock::now();
        printf("[setOctomap] rebuildDerivedLayers: %zu (%.1f ms)\n", traversable_cells_.size(),
               std::chrono::duration<double, std::milli>(t3 - t2).count());

        if (radical_infill_enabled_) {
          radicalInfill();
        }
        auto t4 = std::chrono::steady_clock::now();
        printf("[setOctomap] radicalInfill: (%.1f ms)\n",
               std::chrono::duration<double, std::milli>(t4 - t3).count());

        if (flatten_enabled_) {
          flattenTraversable();
        }
        auto t5 = std::chrono::steady_clock::now();
        printf("[setOctomap] flattenTraversable: (%.1f ms)\n",
               std::chrono::duration<double, std::milli>(t5 - t4).count());

        rebuildPreblockedCostmap();
        auto t6 = std::chrono::steady_clock::now();
        printf("[setOctomap] rebuildPreblockedCostmap: %zu (%.1f ms)\n", preblocked_costmap_.size(),
               std::chrono::duration<double, std::milli>(t6 - t5).count());
    }

    void GlobalPlanner::rebuildFromSnapshot(
        const std::unordered_set<GridIndex, GridIndexHash>& occupied_set)
    {
        if (!octree_) return;
        occupied_set_ = occupied_set;
        initGridLookup();
        rebuildPreblockedCells();
        rebuildDerivedLayers();
        if (radical_infill_enabled_) {
          radicalInfill();
        }
        if (flatten_enabled_) {
          flattenTraversable();
        }
        rebuildPreblockedCostmap();
    }

    void GlobalPlanner::reanalyze()
    {
        if (!octree_ || !map_ready_) return;

        // Build occupied snapshot from octree (needed by rebuild* functions below)
        occupied_set_.clear();
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
            if (octree_->isNodeOccupied(*it))
                occupied_set_.insert(worldToGrid(it.getX(), it.getY(), it.getZ()));
        }
        initGridLookup();

        rebuildPreblockedCells();
        rebuildDerivedLayers();
        if (radical_infill_enabled_) {
          radicalInfill();
        }
        if (flatten_enabled_) {
          flattenTraversable();
        }
        rebuildPreblockedCostmap();
    }

    void GlobalPlanner::makePlan(const PointPose start,const PointPose goal)
    {
        start_point_ = start;
        has_start_ = true;

        goal_point_ = goal;
        has_goal_ = true;

        // printf("start = (%f,%f,%f),goal = (%f,%f,%f) \n",start_point_.x,start_point_.y,start_point_.z,goal_point_.x,goal_point_.y,goal_point_.z);
         
        tryPlan();

    }

    void GlobalPlanner::tryPlan()
    {
        // printf("GlobalPlanner::tryPlan planning...\n");
        if (!map_ready_ || !has_start_ || !has_goal_ || planning_in_progress_) 
        {
            printf("GlobalPlanner::tryPlan 异常退出规划器\n");
            return;
        }
        planning_in_progress_ = true;
        ++plan_seq_;
        const bool ok = startPlan();
        planning_in_progress_ = false;
        if (!ok) 
        {
            printf("GlobalPlanner::tryPlan() A* planning failed. \n");
        } 
        else 
        {
            last_success_seq_ = plan_seq_;
        }
    }

    void GlobalPlanner::getPlannerResults(std::vector<PointPose>& plannerResults)
    {
       plannerResults = planner_results_;
    }

    bool GlobalPlanner::startPlan()
    {
        const double robot_radius = robot_radius_;
        const int max_iterations = max_iterations_;
        const int snap_radius = snap_search_radius_cells_;
        const bool require_ground_support = require_ground_support_; 
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells =  ground_support_xy_radius_cells_;
        const int support_depth_cells = ground_support_depth_cells_;
        const bool enable_preblocked_costmap =  enable_preblocked_costmap_;
        const double preblocked_costmap_weight = preblocked_costmap_weight_;

        const GridIndex start_raw = worldToGrid(
        start_point_.x, start_point_.y, start_point_.z);
        const GridIndex goal_raw = worldToGrid(
        goal_point_.x, goal_point_.y, goal_point_.z);

        GridIndex start = start_raw;
        GridIndex goal = goal_raw;
        const bool start_ok = findNearestFreeCell(
        start_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
        support_xy_radius_cells, support_depth_cells, start);
        const bool goal_ok = findNearestFreeCell(
        goal_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
        support_xy_radius_cells, support_depth_cells, goal);

        if (!start_ok) 
        {
            printf("GlobalPlanner::startPlan() Start is occupied/out of map and no nearby free cell.\n");
            return false;
        }

        if (!goal_ok) 
        {
            printf("GlobalPlanner::startPlan() Goal is occupied/out of map and no nearby free cell.\n");
            return false;
        }

        if (!(start == start_raw)) 
        {
            const auto p = gridToWorld(start);
            // printf("GlobalPlanner::startPlan() Start snapped to free cell: [%.2f, %.2f, %.2f] \n",p.x(), p.y(), p.z());
        }

        if (!(goal == goal_raw))
        {
            const auto p = gridToWorld(goal);
            // printf("GlobalPlanner::startPlan() Goal snapped to free cell: [%.2f, %.2f, %.2f] \n",p.x(), p.y(), p.z());
        }

        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
        std::unordered_map<GridIndex, double, GridIndexHash> g_score;
        std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
        std::unordered_set<GridIndex, GridIndexHash> closed_set;

        g_score[start] = 0.0;
        open_set.push(QueueNode{start, euclidean(start, goal), 0.0});

        const std::vector<GridIndex> directions = make26Directions();
        int iters = 0;

        while (!open_set.empty() && iters < max_iterations
               && (cancel_flag_ == nullptr || !cancel_flag_->load()))
        {
            const QueueNode current = open_set.top();
            open_set.pop();
            ++iters;

            if (closed_set.find(current.idx) != closed_set.end()) {
                continue;
            }
            closed_set.insert(current.idx);

            if (current.idx == goal) {
                const auto cells = reconstructPath(came_from, current.idx);
                // printf("GlobalPlanner::startPlan() A* path found in %d iterations. waypoints=%zu \n", iters, cells.size());
                planner_results_.clear();
                for (std::size_t i = 0; i < cells.size(); ++i) 
                {
                    const auto & c = cells[i];
                    const auto p = gridToWorld(c);
                    PointPose temp;
                    temp.x = p.x();
                    temp.y = p.y();
                    temp.z = p.z();
                    planner_results_.push_back(temp);
                }

                return true;
            }

            for (const auto & d : directions) 
            {
                GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
                if (closed_set.find(nbr) != closed_set.end()) {
                continue;
                }
                if (!isCellTraversable(
                    nbr, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                continue;
                }
                const double step_cost = euclidean(current.idx, nbr);
                double tentative_g = current.g + step_cost;
                if (enable_preblocked_costmap) {
                tentative_g += preblocked_costmap_weight * getPreblockedCost(nbr);
                }

                auto g_it = g_score.find(nbr);
                if (g_it == g_score.end() || tentative_g < g_it->second) {
                came_from[nbr] = current.idx;
                g_score[nbr] = tentative_g;
                const double f = tentative_g + euclidean(nbr, goal);
                open_set.push(QueueNode{nbr, f, tentative_g});
                }
            }
        }

        return false;
    }

    std::vector<GridIndex> GlobalPlanner::reconstructPath(const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,GridIndex current) const
    {
        std::vector<GridIndex> path;
        path.push_back(current);
        while (came_from.find(current) != came_from.end()) 
        {
            current = came_from.at(current);
            path.push_back(current);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    bool GlobalPlanner::findNearestFreeCell(const GridIndex & seed, double robot_radius, int radius_cells, bool require_ground_support,bool strict_direct_ground_support, int support_xy_radius_cells, int support_depth_cells,GridIndex & out) const
    {
        if (isCellTraversable(
            seed, robot_radius, require_ground_support, strict_direct_ground_support,
            support_xy_radius_cells, support_depth_cells))
        {
        out = seed;
        return true;
        }

        for (int r = 1; r <= radius_cells; ++r) {
        for (int dz = 0; dz <= r; ++dz) {
            for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
                continue;
                }

                GridIndex c1{seed.x + dx, seed.y + dy, seed.z + dz};
                if (isCellTraversable(
                    c1, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                out = c1;
                return true;
                }

                if (dz > 0) {
                GridIndex c2{seed.x + dx, seed.y + dy, seed.z - dz};
                if (isCellTraversable(
                    c2, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                    out = c2;
                    return true;
                }
                }
            }
            }
        }
        }
        return false;
    }

    double GlobalPlanner::getPreblockedCost(const GridIndex & idx) const
    {
        const auto it = preblocked_costmap_.find(idx);
        if (it == preblocked_costmap_.end()) {
        return 0.0;
        }
        return it->second;
    }

    std::vector<GridIndex> GlobalPlanner::make26Directions() const
    {
        std::vector<GridIndex> dirs;
        dirs.reserve(26);
        for (int dx = -1; dx <= 1; ++dx) 
        {
            for (int dy = -1; dy <= 1; ++dy) 
            {
                for (int dz = -1; dz <= 1; ++dz) 
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }
                    dirs.push_back(GridIndex{dx, dy, dz});
                }
            }
        }
        return dirs;
    }

    bool GlobalPlanner::isCellTraversable(const GridIndex & idx, double robot_radius, bool require_ground_support,bool strict_direct_ground_support,int support_xy_radius_cells, int support_depth_cells) const
    {
        if (!isInsideMetricBounds(idx)) {
        return false;
        }

        if (require_ground_support &&
        !hasGroundSupport(
            idx, strict_direct_ground_support, support_xy_radius_cells, support_depth_cells))
        {
        return false;
        }

        if (preblocked_hard_obstacle_) {
        for (int z = idx.z - 1; z >= 0; --z) {
            const GridIndex below_idx{idx.x, idx.y, z};
            if (isOccupiedCell(below_idx)) {
            break;
            }
            if (grid_lookup_.inBounds(below_idx) && grid_lookup_.testFlags(below_idx, FLAG_PREBLOCKED)) {
            return false;
            }
        }
        }

        // Direct grid arithmetic — avoid worldToGrid roundtrip (3 divisions per neighbor)
        const double r = cached_resolution_;
        const int n = std::max(1, static_cast<int>(std::ceil(robot_radius / r)));
        const double radius_sq = robot_radius * robot_radius;

        for (int dx = -n; dx <= n; ++dx) {
        for (int dy = -n; dy <= n; ++dy) {
            for (int dz = 0; dz <= n; ++dz) {
            const double dist_x = static_cast<double>(dx) * r;
            const double dist_y = static_cast<double>(dy) * r;
            const double dist_z = static_cast<double>(dz) * r;
            const double dist_sq = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
            if (dist_sq > radius_sq) {
                continue;
            }
            const GridIndex nearby_idx{idx.x + dx, idx.y + dy, idx.z + dz};
            if (!grid_lookup_.inBounds(nearby_idx)) {
                return false;
            }
            if (preblocked_hard_obstacle_ &&
                grid_lookup_.testFlags(nearby_idx, FLAG_PREBLOCKED)) {
                return false;
            }
            if (grid_lookup_.testFlags(nearby_idx, FLAG_OCCUPIED)) {
                return false;
            }
            }
        }
        }
        return true;
    }

    bool GlobalPlanner::hasGroundSupport(const GridIndex & idx, bool strict_direct_ground_support, int support_xy_radius_cells,int support_depth_cells) const
    {
        if (strict_direct_ground_support) {
        GridIndex below{idx.x, idx.y, idx.z - 1};
        return isOccupiedCell(below);
        }

        for (int dz = 1; dz <= std::max(1, support_depth_cells); ++dz) {
        for (int dx = -support_xy_radius_cells; dx <= support_xy_radius_cells; ++dx) {
            for (int dy = -support_xy_radius_cells; dy <= support_xy_radius_cells; ++dy) {
            GridIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
            if (isOccupiedCell(below)) {
                return true;
            }
            }
        }
        }
        return false;
    }

    void GlobalPlanner::initGridLookup()
    {
        if (occupied_set_.empty()) return;

        // Compute grid bounding box from occupied cells
        int gmin_x = INT_MAX, gmin_y = INT_MAX, gmin_z = INT_MAX;
        int gmax_x = INT_MIN, gmax_y = INT_MIN, gmax_z = INT_MIN;
        for (const auto & g : occupied_set_) {
            if (g.x < gmin_x) gmin_x = g.x;
            if (g.x > gmax_x) gmax_x = g.x;
            if (g.y < gmin_y) gmin_y = g.y;
            if (g.y > gmax_y) gmax_y = g.y;
            if (g.z < gmin_z) gmin_z = g.z;
            if (g.z > gmax_z) gmax_z = g.z;
        }

        // Pad grid to cover all neighbor lookups without bounds-check overhead
        const int n_robot = std::max(1, static_cast<int>(std::ceil(robot_radius_ / cached_resolution_)));
        const int pad = std::max({
            snap_search_radius_cells_ + 1,
            preblocked_costmap_radius_cells_ + 1,
            n_robot + 1,
            ground_support_xy_radius_cells_ + 1,
            4
        });
        const int sx = gmax_x - gmin_x + 1 + 2 * pad;
        const int sy = gmax_y - gmin_y + 1 + 2 * pad;
        const int sz = gmax_z - gmin_z + 1 + 2 * pad;
        grid_lookup_.resize(gmin_x - pad, gmin_y - pad, gmin_z - pad, sx, sy, sz);

        for (const auto & g : occupied_set_) {
            grid_lookup_.setFlags(g, FLAG_OCCUPIED);
        }
    }

    void GlobalPlanner::syncTraversableFlags()
    {
        grid_lookup_.clearFlags(FLAG_TRAVERSABLE);
        for (const auto & c : traversable_cells_) {
            grid_lookup_.setFlags(c, FLAG_TRAVERSABLE);
        }
    }

    void GlobalPlanner::rebuildPreblockedCells()
    {
        preblocked_cells_.clear();
        if (occupied_set_.empty()) {
        return;
        }

        // Use flat grid FLAG_CANDIDATE for dedup instead of unordered_set (vector + flag is ~100x faster)
        grid_lookup_.clearFlags(FLAG_CANDIDATE);
        std::vector<GridIndex> candidates;
        candidates.reserve(occupied_set_.size() * 6);
        for (const auto & occ : occupied_set_) {
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            const GridIndex c{occ.x + dx, occ.y + dy, occ.z};
            if (!grid_lookup_.inBounds(c)) continue;
            if (grid_lookup_.testFlags(c, FLAG_OCCUPIED | FLAG_CANDIDATE)) continue;
            grid_lookup_.setFlags(c, FLAG_CANDIDATE);
            candidates.push_back(c);
            }
        }
        }

        for (const auto & c : candidates) {
        if (isOccupiedCell(c)) {
            continue;
        }
        const GridIndex below0{c.x, c.y, c.z - 1};
        const bool below0_occ = isInsideMetricBounds(below0) && isOccupiedCell(below0);
        if (below0_occ && hasSameLevelNeighborWithOccupiedAbove(c)) {
            preblocked_cells_.insert(c);
            continue;
        }
        const GridIndex above1{c.x, c.y, c.z + 1};
        const bool above1_occ = isInsideMetricBounds(above1) && isOccupiedCell(above1);
        if (!hasNonOccupiedNeighborSameLevel(c)) {
            continue;
        }
        if (above1_occ) {
            continue;
        }
        const GridIndex below1{c.x, c.y, c.z - 1};
        if (!isInsideMetricBounds(below1)) {
            continue;
        }
        const bool below1_non_occupied = !isOccupiedCell(below1);
        if (below1_non_occupied) {
            preblocked_cells_.insert(c);
        }
        }

        for (const auto & c : external_preblocked_cells_) {
        if (isInsideMetricBounds(c) && !isOccupiedCell(c)) {
            preblocked_cells_.insert(c);
        }
        }

        // Clean up temp candidate flags + sync preblocked flags to flat grid
        if (!grid_lookup_.empty()) {
            grid_lookup_.clearFlags(FLAG_CANDIDATE);
            grid_lookup_.clearFlags(FLAG_PREBLOCKED);
            for (const auto & c : preblocked_cells_) {
                grid_lookup_.setFlags(c, FLAG_PREBLOCKED);
            }
        }
        // printf("Preprocess mask rebuilt. preblocked_cells=%zu external=%zu \n",preblocked_cells_.size(), external_preblocked_cells_.size());
        // publishPreblockedCellsMarker();
    }

    bool GlobalPlanner::hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const
    {
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
            continue;
            }
            const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
            if (!isInsideMetricBounds(n)) {
            continue;
            }
            const GridIndex n_above1{n.x, n.y, n.z + 1};
            if (!isInsideMetricBounds(n_above1)) {
            continue;
            }
            if (isOccupiedCell(n_above1)) {
            return true;
            }
        }
        }
        return false;
    }

    bool GlobalPlanner::hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const
    {
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
            continue;
            }
            const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
            if (!isInsideMetricBounds(n)) {
            continue;
            }
            if (!isOccupiedCell(n)) {
            return true;
            }
        }
        }
        return false;
    }

    void GlobalPlanner::rebuildDerivedLayers()
    {
        traversable_cells_.clear();
        if (occupied_set_.empty()) {
        return;
        }

        const bool require_ground_support = require_ground_support_;
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells = ground_support_xy_radius_cells_;
        const int support_depth_cells = ground_support_depth_cells_;
        const double robot_radius = robot_radius_;
        const bool lowest_traversable_only = lowest_traversable_only_;

        // Cells that could possibly be traversable must have an occupied cell below.
        // Instead of scanning the entire bbox, iterate only occupied cells and check
        // the candidates above them.
        const int max_dz_cand = require_ground_support
            ? (strict_direct_ground_support ? 1 : support_depth_cells)
            : 1;
        const int max_xy_cand = require_ground_support
            ? (strict_direct_ground_support ? 0 : support_xy_radius_cells)
            : 0;

        // Use flat grid FLAG_CHECKED for dedup instead of unordered_set
        grid_lookup_.clearFlags(FLAG_CHECKED);

        // Per-(x,y) set for lowest_traversable_only fast-path
        std::unordered_set<uint64_t> columns_done;

        for (const auto & occ : occupied_set_) {
            for (int dx = -max_xy_cand; dx <= max_xy_cand; ++dx) {
                for (int dy = -max_xy_cand; dy <= max_xy_cand; ++dy) {
                    for (int dz = 1; dz <= max_dz_cand; ++dz) {
                        const GridIndex candidate{occ.x + dx, occ.y + dy, occ.z + dz};

                        if (grid_lookup_.testFlags(candidate, FLAG_CHECKED)) continue;
                        if (lowest_traversable_only) {
                            uint64_t col = (static_cast<uint64_t>(static_cast<uint32_t>(candidate.x)) << 32)
                                         | static_cast<uint32_t>(candidate.y);
                            if (columns_done.count(col)) continue;
                        }
                        if (!grid_lookup_.inBounds(candidate)) continue;
                        if (grid_lookup_.testFlags(candidate, FLAG_OCCUPIED | FLAG_CHECKED)) continue;

                        grid_lookup_.setFlags(candidate, FLAG_CHECKED);

                        if (isCellTraversable(
                            candidate, robot_radius, require_ground_support,
                            strict_direct_ground_support,
                            support_xy_radius_cells, support_depth_cells))
                        {
                            traversable_cells_.insert(candidate);
                            if (lowest_traversable_only) {
                                uint64_t col = (static_cast<uint64_t>(static_cast<uint32_t>(candidate.x)) << 32)
                                             | static_cast<uint32_t>(candidate.y);
                                columns_done.insert(col);
                            }
                        }
                    }
                }
            }
        }

        // Clean up temp checked flags + sync traversable flags to flat grid
        if (!grid_lookup_.empty()) {
            grid_lookup_.clearFlags(FLAG_CHECKED);
            syncTraversableFlags();
        }
        // printf("Traversable cells rebuilt: %zu cells (lowest_only=%d)\n",
        //        traversable_cells_.size(), lowest_traversable_only ? 1 : 0);
    }

    bool GlobalPlanner::isInsideMetricBounds(const GridIndex & idx) const
    {
        // Fast path: integer grid bounds comparison (6 int compares, no float math)
        if (!grid_lookup_.empty()) {
            return grid_lookup_.inBounds(idx);
        }
        // Fallback before grid is initialized
        const auto p = gridToWorld(idx);
        return p.x() >= static_cast<float>(metric_min_x_) && p.x() <= static_cast<float>(metric_max_x_) &&
            p.y() >= static_cast<float>(metric_min_y_) && p.y() <= static_cast<float>(metric_max_y_) &&
            p.z() >= static_cast<float>(metric_min_z_) && p.z() <= static_cast<float>(metric_max_z_);
    }

    bool GlobalPlanner::isOccupiedCell(const GridIndex & idx) const
    {
        if (!grid_lookup_.inBounds(idx)) {
        return false;
        }
        return grid_lookup_.testFlags(idx, FLAG_OCCUPIED);
    }

    GridIndex GlobalPlanner::worldToGrid(double x, double y, double z) const
    {
        const double r = cached_resolution_;
        return GridIndex{
        static_cast<int>(std::floor(x / r)),
        static_cast<int>(std::floor(y / r)),
        static_cast<int>(std::floor(z / r))};
    }

    octomap::point3d GlobalPlanner::gridToWorld(const GridIndex & idx) const
    {
        const double r = cached_resolution_;
        return octomap::point3d(
        static_cast<float>((static_cast<double>(idx.x) + 0.5) * r),
        static_cast<float>((static_cast<double>(idx.y) + 0.5) * r),
        static_cast<float>((static_cast<double>(idx.z) + 0.5) * r));
    }

    void GlobalPlanner::rebuildPreblockedCostmap()
    {
        preblocked_costmap_.clear();
        if (!octree_) {
        return;
        }
        const bool enable = enable_preblocked_costmap_;
        if (!enable) {
        return;
        }

        const int radius_cells = std::max(
        1, static_cast<int>(preblocked_costmap_radius_cells_));
        const double denom = static_cast<double>(radius_cells) + 1.0;

        // Precompute sphere offsets (within radius, excluding center).
        // ~113 offsets vs 343 for full cube -- avoids 67% of find() calls.
        struct Offset { int dx, dy, dz; double cost; };
        std::vector<Offset> offsets;
        offsets.reserve(150);
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
              if (dx == 0 && dy == 0 && dz == 0) continue;
              const double d = std::sqrt(
                static_cast<double>(dx * dx + dy * dy + dz * dz));
              if (d > static_cast<double>(radius_cells)) continue;
              offsets.push_back({dx, dy, dz, std::max(0.0, (denom - d) / denom)});
            }
          }
        }

        // Iterate traversable cells, check sphere of neighbors for preblocked.
        for (const auto & t : traversable_cells_) {
          if (grid_lookup_.testFlags(t, FLAG_PREBLOCKED)) {
            continue;
          }
          double best_cst = 0.0;
          for (const auto & off : offsets) {
            const GridIndex n{t.x + off.dx, t.y + off.dy, t.z + off.dz};
            if (!grid_lookup_.testFlags(n, FLAG_PREBLOCKED)) {
              continue;
            }
            if (off.cost > best_cst) {
              best_cst = off.cost;
            }
          }
          if (best_cst > 0.0) {
            preblocked_costmap_[t] = best_cst;
          }
        }

        // RCLCPP_INFO(
        // get_logger(),
        // "Preblocked costmap rebuilt. cells=%zu radius=%d",
        // preblocked_costmap_.size(), radius_cells);
        // printf("Preblocked costmap rebuilt. cells=%zu radius=%d \n",preblocked_costmap_.size(),radius_cells);
        // publishRiskCostCloud();
    }

    void GlobalPlanner::radicalInfill()
    {
        if (!octree_ || traversable_cells_.empty()) {
            return;
        }

        const double res = cached_resolution_;
        const int radius_cells = std::max(1,
            static_cast<int>(std::ceil(radical_infill_radius_m_ / res)));
        const int half_z_cells = std::max(1,
            static_cast<int>(std::ceil(radical_infill_half_height_m_ / res)));
        const int clearance_cells = std::max(1,
            static_cast<int>(std::ceil(radical_infill_clearance_m_ / res)));

        // Step 1: 26-connected components of traversable_cells_
        // Use flat int array for component_id instead of unordered_map (~100x faster)
        const auto & gl = grid_lookup_;
        std::vector<int> comp_grid(gl.data.size(), -1);
        const std::vector<GridIndex> dirs_26 = make26Directions();
        std::vector<std::vector<GridIndex>> components;

        for (const auto & cell : traversable_cells_) {
            if (!gl.inBounds(cell)) continue;
            const size_t fi = gl.flatIndex(cell);
            if (comp_grid[fi] >= 0) continue;
            const int comp_idx = static_cast<int>(components.size());
            std::vector<GridIndex> comp;
            std::queue<GridIndex> q;
            q.push(cell);
            comp_grid[fi] = comp_idx;
            while (!q.empty()) {
                const GridIndex cur = q.front();
                q.pop();
                comp.push_back(cur);
                for (const auto & d : dirs_26) {
                    const GridIndex nbr{cur.x + d.x, cur.y + d.y, cur.z + d.z};
                    if (!gl.inBounds(nbr)) continue;
                    const size_t nfi = gl.flatIndex(nbr);
                    if (!gl.testFlags(nbr, FLAG_TRAVERSABLE)) continue;
                    if (comp_grid[nfi] >= 0) continue;
                    comp_grid[nfi] = comp_idx;
                    q.push(nbr);
                }
            }
            components.push_back(std::move(comp));
        }

        // printf("RadicalInfill: %zu components from %zu traversable cells.\n",
        //        components.size(), traversable_cells_.size());

        if (components.size() < 2) {
            // printf("RadicalInfill: nothing to bridge.\n");
            return;
        }

        // Find edge cells per component (limit per component)
        const size_t max_edge_per_comp = 500;
        std::vector<std::vector<GridIndex>> edge_cells(components.size());
        for (size_t ci = 0; ci < components.size(); ++ci) {
            for (const auto & cell : components[ci]) {
                if (edge_cells[ci].size() >= max_edge_per_comp) break;
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        if (dx == 0 && dy == 0) continue;
                        const GridIndex nbr{cell.x + dx, cell.y + dy, cell.z};
                        if (!grid_lookup_.testFlags(nbr, FLAG_TRAVERSABLE)) {
                            edge_cells[ci].push_back(cell);
                            goto next_cell;
                        }
                    }
                }
            next_cell:;
            }
        }

        // Use flat bit array for filled cells instead of unordered_set
        std::vector<bool> filled_grid(gl.data.size(), false);

        for (size_t ci = 0; ci < components.size(); ++ci) {
            for (const auto & seed : edge_cells[ci]) {
                for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                        if (dx * dx + dy * dy > radius_cells * radius_cells) continue;
                        for (int dz = -half_z_cells; dz <= half_z_cells; ++dz) {
                            const GridIndex target{seed.x + dx, seed.y + dy, seed.z + dz};
                            if (!gl.inBounds(target)) continue;
                            if (!gl.testFlags(target, FLAG_TRAVERSABLE)) continue;
                            const size_t tfi = gl.flatIndex(target);
                            const int cj = comp_grid[tfi];
                            if (cj < 0 || static_cast<size_t>(cj) == ci) continue;

                            // Bresenham 2D line from seed to target, Z-interpolated
                            const int x0 = seed.x, y0 = seed.y;
                            const int x1 = target.x, y1 = target.y;
                            const int dx_abs = std::abs(x1 - x0);
                            const int dy_abs = std::abs(y1 - y0);
                            const int sx = (x0 < x1) ? 1 : -1;
                            const int sy = (y0 < y1) ? 1 : -1;
                            const int steps = std::max(dx_abs, dy_abs) + 1;
                            int err = dx_abs - dy_abs;
                            int cx = x0, cy = y0;

                            for (int step = 0; step < steps; ++step) {
                                const double t = (steps <= 1) ? 0.0 :
                                    static_cast<double>(step) / static_cast<double>(steps - 1);
                                const int cz = static_cast<int>(
                                    std::round(static_cast<double>(seed.z) +
                                        t * static_cast<double>(target.z - seed.z)));

                                const GridIndex lc{cx, cy, cz};

                                if (!gl.inBounds(lc)) goto step_advance;
                                {
                                    const size_t lfi = gl.flatIndex(lc);
                                    if (gl.testFlags(lc, FLAG_TRAVERSABLE) || filled_grid[lfi]) {
                                        goto step_advance;
                                    }
                                    if (isOccupiedCell(lc)) {
                                        goto step_advance;
                                    }
                                    bool cell_blocked = false;
                                    for (int dz_chk = 1; dz_chk <= clearance_cells; ++dz_chk) {
                                        const GridIndex above{lc.x, lc.y, lc.z + dz_chk};
                                        if (isOccupiedCell(above)) {
                                            cell_blocked = true;
                                            break;
                                        }
                                    }
                                    if (!cell_blocked) {
                                        filled_grid[lfi] = true;
                                    }
                                }

                            step_advance:
                                if (cx == x1 && cy == y1) break;
                                const int e2 = 2 * err;
                                if (e2 > -dy_abs) { err -= dy_abs; cx += sx; }
                                if (e2 < dx_abs)  { err += dx_abs; cy += sy; }
                            }
                        }
                    }
                }
            }
        }

        // printf("RadicalInfill: %zu components, filled %zu cells.\n",
        //        components.size(), filled.size());
        // Collect filled cells and add to traversable set + grid flags
        for (size_t i = 0; i < filled_grid.size(); ++i) {
            if (filled_grid[i]) {
                // Decode flat index back to GridIndex
                const int total_xy = gl.size_x * gl.size_y;
                const int lz = static_cast<int>(i / total_xy);
                const int rem = static_cast<int>(i % total_xy);
                const int ly = rem / gl.size_x;
                const int lx = rem % gl.size_x;
                GridIndex fc{gl.origin_x + lx, gl.origin_y + ly, gl.origin_z + lz};
                traversable_cells_.insert(fc);
                grid_lookup_.setFlags(fc, FLAG_TRAVERSABLE);
            }
        }
    }

    void GlobalPlanner::flattenTraversable()
    {
        if (traversable_cells_.empty()) return;

        const int window = flatten_window_cells_;
        const int max_delta = flatten_max_delta_cells_;
        const auto & gl = grid_lookup_;

        // Build 2D height map using flat arrays instead of std::map
        // Sentinel value INT_MIN means "no traversable cell at this (x,y)"
        std::vector<int> height_grid(
            static_cast<size_t>(gl.size_x) * static_cast<size_t>(gl.size_y), INT_MIN);
        auto hidx = [&](int x, int y) -> size_t {
            return static_cast<size_t>(y - gl.origin_y) * static_cast<size_t>(gl.size_x) +
                   static_cast<size_t>(x - gl.origin_x);
        };

        for (const auto& cell : traversable_cells_) {
            if (!gl.inBounds({cell.x, cell.y, gl.origin_z})) continue;
            const size_t hi = hidx(cell.x, cell.y);
            if (height_grid[hi] == INT_MIN || cell.z < height_grid[hi]) {
                height_grid[hi] = cell.z;
            }
        }

        // Collect populated columns for iteration
        std::vector<std::pair<size_t, int>> columns;
        for (size_t hi = 0; hi < height_grid.size(); ++hi) {
            if (height_grid[hi] != INT_MIN) {
                columns.emplace_back(hi, height_grid[hi]);
            }
        }

        // Median-filter each column
        std::vector<std::pair<GridIndex, int>> adjustments;
        for (const auto& [hi, z] : columns) {
            const int x = gl.origin_x + static_cast<int>(hi % gl.size_x);
            const int y = gl.origin_y + static_cast<int>(hi / gl.size_x);

            std::vector<int> neighbor_zs;
            for (int dx = -window; dx <= window; ++dx) {
                for (int dy = -window; dy <= window; ++dy) {
                    const int nx = x + dx, ny = y + dy;
                    if (nx < gl.origin_x || nx >= gl.origin_x + gl.size_x ||
                        ny < gl.origin_y || ny >= gl.origin_y + gl.size_y) continue;
                    const int nz = height_grid[hidx(nx, ny)];
                    if (nz != INT_MIN) {
                        neighbor_zs.push_back(nz);
                    }
                }
            }
            if (neighbor_zs.empty()) continue;

            const size_t mid = neighbor_zs.size() / 2;
            std::nth_element(neighbor_zs.begin(),
                             neighbor_zs.begin() + static_cast<long>(mid),
                             neighbor_zs.end());
            const int median_z = neighbor_zs[mid];

            if (std::abs(z - median_z) <= max_delta && z != median_z) {
                adjustments.emplace_back(GridIndex{x, y, z}, median_z);
            }
        }

        for (const auto& [old_cell, new_z] : adjustments) {
            traversable_cells_.erase(old_cell);
            GridIndex new_cell{old_cell.x, old_cell.y, new_z};
            if (!isOccupiedCell(new_cell)) {
                traversable_cells_.insert(new_cell);
            }
        }

        // printf("FlattenTraversable: adjusted %zu / %zu columns (window=%d, max_delta=%d)\n",
        //        adjustments.size(), height_map.size(), window, max_delta);
    }

}
