
#include "pcd2octomap_converter.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <unordered_set>

#include <pcl/io/pcd_io.h>

namespace pcd2octomap
{

bool Key::operator==(const Key & other) const
{
  return k[0] == other.k[0] && k[1] == other.k[1] && k[2] == other.k[2];
}

std::size_t KeyHash::operator()(const Key & key) const
{
  std::size_t seed = std::hash<unsigned int>{}(key.k[0]);
  seed ^= std::hash<unsigned int>{}(key.k[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<unsigned int>{}(key.k[2]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

Pcd2OctomapConverter::Pcd2OctomapConverter()
: cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  tree_(std::make_shared<octomap::OcTree>(resolution_))
{
}

void Pcd2OctomapConverter::configure(const ConverterConfig& config)
{
  input_pcd_ = config.input_pcd;
  output_bt_ = config.output_bt;
  resolution_ = config.resolution;
  min_points_per_voxel_ = config.min_points_per_voxel;
  min_cluster_voxels_ = config.min_cluster_voxels;
  save_to_file_ = config.save_to_file;
  enable_ground_infill_ = config.enable_ground_infill;
  ground_infill_density_threshold_ = config.ground_infill_density_threshold;
  ground_infill_neighbor_threshold_ = config.ground_infill_neighbor_threshold;
  tree_ = std::make_shared<octomap::OcTree>(resolution_);
}

bool Pcd2OctomapConverter::convert()
{
  if (!loadPointCloud()) {
    return false;
  }

  tree_ = std::make_shared<octomap::OcTree>(resolution_);

  buildVoxelCounts();
  filterByPointCount();
  filterByConnectedClusters();
  fillOcTree();

  if (enable_ground_infill_) {
    std::size_t before_count = 0;
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (tree_->isNodeOccupied(*it)) ++before_count;
    }
    groundInfill();
    std::size_t after_count = 0;
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (tree_->isNodeOccupied(*it)) ++after_count;
    }
    std::cout << "Ground infill result: " << before_count << " -> " << after_count
              << " occupied leaves (+" << (after_count - before_count) << ")" << std::endl;
  }

  if (save_to_file_) {
    if (!saveOctomap()) {
      return false;
    }
    std::cout << "\nConversion finished. To visualize, run:\n";
    std::cout << "octovis " << output_bt_ << std::endl;
  }

  return true;
}

bool Pcd2OctomapConverter::loadPointCloud()
{
  cloud_->clear();

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_, *cloud_) == -1) {
    std::cerr << "Couldn't read file " << input_pcd_ << std::endl;
    return false;
  }

  std::cout << "Loaded " << cloud_->size() << " points." << std::endl;
  return true;
}

void Pcd2OctomapConverter::buildVoxelCounts()
{
  voxel_counts_.clear();

  for (const auto & p : cloud_->points) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      continue;
    }

    octomap::OcTreeKey raw_key;
    if (tree_->coordToKeyChecked(p.x, p.y, p.z, raw_key)) {
      Key key{{raw_key.k[0], raw_key.k[1], raw_key.k[2]}};
      ++voxel_counts_[key];
    }
  }
}

void Pcd2OctomapConverter::filterByPointCount()
{
  occupied_keys_.clear();

  for (const auto & entry : voxel_counts_) {
    if (static_cast<int>(entry.second) >= min_points_per_voxel_) {
      occupied_keys_.insert(entry.first);
    }
  }

  std::cout << "Voxels after point-count filtering: "
            << occupied_keys_.size() << std::endl;
}

void Pcd2OctomapConverter::filterByConnectedClusters()
{
  if (min_cluster_voxels_ <= 1 || occupied_keys_.empty()) {
    std::cout << "Voxels after cluster filtering: "
              << occupied_keys_.size() << std::endl;
    return;
  }

  std::unordered_set<Key, KeyHash> filtered_keys;
  std::unordered_set<Key, KeyHash> visited;
  filtered_keys.reserve(occupied_keys_.size());

  for (const auto & seed : occupied_keys_) {
    if (visited.count(seed)) {
      continue;
    }

    std::deque<Key> queue;
    std::vector<Key> cluster;

    queue.push_back(seed);
    visited.insert(seed);

    while (!queue.empty()) {
      Key current = queue.front();
      queue.pop_front();
      cluster.push_back(current);

      // 检查 26 邻域，3x3x3 范围
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }

            Key neighbor{{
              static_cast<unsigned int>(current.k[0] + dx),
              static_cast<unsigned int>(current.k[1] + dy),
              static_cast<unsigned int>(current.k[2] + dz)
            }};

            if (occupied_keys_.count(neighbor) && !visited.count(neighbor)) {
              visited.insert(neighbor);
              queue.push_back(neighbor);
            }
          }
        }
      }
    }

    if (static_cast<int>(cluster.size()) >= min_cluster_voxels_) {
      for (const auto & key : cluster) {
        filtered_keys.insert(key);
      }
    }
  }

  occupied_keys_ = std::move(filtered_keys);

  std::cout << "Voxels after cluster filtering: "
            << occupied_keys_.size() << std::endl;
}

void Pcd2OctomapConverter::fillOcTree()
{
  for (const auto & key : occupied_keys_) {
    octomap::OcTreeKey octo_key;
    octo_key.k[0] = key.k[0];
    octo_key.k[1] = key.k[1];
    octo_key.k[2] = key.k[2];

    tree_->updateNode(tree_->keyToCoord(octo_key), true);
  }

  tree_->updateInnerOccupancy();
}

void Pcd2OctomapConverter::groundInfill()
{
  if (!tree_) return;

  const double res = tree_->getResolution();

  // Collect all occupied leaf cells into a grid indexed by (ix, iy, iz)
  struct CellKey { int x, y, z; };
  struct CellKeyHash {
    std::size_t operator()(const CellKey& k) const {
      std::size_t h = std::hash<int>{}(k.x);
      h ^= std::hash<int>{}(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= std::hash<int>{}(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
      return h;
    }
  };
  struct CellKeyEq {
    bool operator()(const CellKey& a, const CellKey& b) const {
      return a.x == b.x && a.y == b.y && a.z == b.z;
    }
  };

  // Build 3D occupied set
  std::unordered_set<CellKey, CellKeyHash, CellKeyEq> occupied;
  int min_iz = std::numeric_limits<int>::max();
  int max_iz = std::numeric_limits<int>::min();
  int min_ix = std::numeric_limits<int>::max(), max_ix = std::numeric_limits<int>::min();
  int min_iy = std::numeric_limits<int>::max(), max_iy = std::numeric_limits<int>::min();

  for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
    if (!tree_->isNodeOccupied(*it)) continue;
    int ix = static_cast<int>(std::floor(it.getX() / res));
    int iy = static_cast<int>(std::floor(it.getY() / res));
    int iz = static_cast<int>(std::floor(it.getZ() / res));
    occupied.insert({ix, iy, iz});
    if (iz < min_iz) min_iz = iz;
    if (iz > max_iz) max_iz = iz;
    if (ix < min_ix) min_ix = ix;
    if (ix > max_ix) max_ix = ix;
    if (iy < min_iy) min_iy = iy;
    if (iy > max_iy) max_iy = iy;
  }

  const int nx = max_ix - min_ix + 1;
  const int ny = max_iy - min_iy + 1;
  const int total_xy = nx * ny;
  const int neighbor_thres = ground_infill_neighbor_threshold_;
  const double density_thres = ground_infill_density_threshold_;

  int total_infill = 0;

  // For each Z layer, check if it's a floor-like layer and fill gaps
  for (int iz = min_iz; iz <= max_iz; ++iz) {
    // Count occupied cells in this layer
    int layer_count = 0;
    for (int ix = min_ix; ix <= max_ix; ++ix) {
      for (int iy = min_iy; iy <= max_iy; ++iy) {
        if (occupied.count({ix, iy, iz})) {
          ++layer_count;
        }
      }
    }

    double density = static_cast<double>(layer_count) / total_xy;
    if (density < density_thres) continue;

    // This is a floor-like layer - fill gaps where enough neighbors exist
    std::vector<CellKey> to_fill;
    for (int ix = min_ix; ix <= max_ix; ++ix) {
      for (int iy = min_iy; iy <= max_iy; ++iy) {
        if (occupied.count({ix, iy, iz})) continue;

        // Count 8-connected neighbors in same Z layer
        int nbr_count = 0;
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            if (occupied.count({ix + dx, iy + dy, iz})) {
              ++nbr_count;
            }
          }
        }

        if (nbr_count >= neighbor_thres) {
          to_fill.push_back({ix, iy, iz});
        }
      }
    }

    // Insert infill cells into the OcTree
    for (const auto& cell : to_fill) {
      double wx = (static_cast<double>(cell.x) + 0.5) * res;
      double wy = (static_cast<double>(cell.y) + 0.5) * res;
      double wz = (static_cast<double>(cell.z) + 0.5) * res;
      tree_->updateNode(octomap::point3d(wx, wy, wz), true);
      occupied.insert(cell);
    }
    total_infill += static_cast<int>(to_fill.size());
  }

  tree_->updateInnerOccupancy();
  std::cout << "Ground infill: added " << total_infill << " voxels" << std::endl;
}

bool Pcd2OctomapConverter::saveOctomap() const
{
  if (!tree_) {
    std::cerr << "OcTree is null, cannot save." << std::endl;
    return false;
  }

  if (tree_->writeBinary(output_bt_)) {
    std::cout << "Success! Saved to " << output_bt_ << std::endl;
    return true;
  }

  std::cerr << "Failed to save " << output_bt_ << std::endl;
  return false;
}

bool Pcd2OctomapConverter::isPointFree(const octomap::point3d & p) const
{
  if (!tree_) {
    return false;
  }

  octomap::OcTreeNode * node = tree_->search(p);

  // 未知区域按可通行处理
  if (node == nullptr) {
    return true;
  }

  return !tree_->isNodeOccupied(node);
}

bool Pcd2OctomapConverter::isSpaceFree(
  const octomap::point3d & min_pt,
  const octomap::point3d & max_pt) const
{
  if (!tree_) {
    return false;
  }

  for (octomap::OcTree::leaf_bbx_iterator it = tree_->begin_leafs_bbx(min_pt, max_pt),
       end = tree_->end_leafs_bbx(); it != end; ++it)
  {
    if (tree_->isNodeOccupied(*it)) {
      return false;
    }
  }

  return true;
}

std::shared_ptr<octomap::OcTree> Pcd2OctomapConverter::getOctomap()
{
    return tree_;
}

void Pcd2OctomapConverter::printOccupiedNodes() const
{
  if (!tree_) {
    return;
  }

  int count = 0;

  for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs(),
       end = tree_->end_leafs(); it != end; ++it)
  {
    if (tree_->isNodeOccupied(*it)) {
      octomap::point3d p = it.getCoordinate();
      double size = it.getSize();

      std::cout << "Node [" << count << "]: "
                << "x=" << p.x() << ", y=" << p.y() << ", z=" << p.z()
                << " (Size: " << size << ")" << std::endl;

      ++count;
    }
  }
}

void Pcd2OctomapConverter::printQueryExamples() const
{
  octomap::point3d current_pos(13.1, 4.1, 14);

  if (isPointFree(current_pos)) {
    std::cout << "free" << std::endl;
  } else {
    std::cout << "occupy" << std::endl;
  }

  octomap::point3d min_pt(13.1, 4.1, 10);
  octomap::point3d max_pt(13.1, 4.1, 16);

  if (isSpaceFree(min_pt, max_pt)) {
    std::cout << "free" << std::endl;
  } else {
    std::cout << "occupancy" << std::endl;
  }
}

void Pcd2OctomapConverter::visualizeWithOctovis() const
{
  std::system(("octovis " + output_bt_).c_str());
}

std::shared_ptr<octomap::OcTree> Pcd2OctomapConverter::getTree() const
{
  return tree_;
}

}  // namespace pcd2octomap

// int main()
// {
//   pcd2octomap::Pcd2OctomapConverter converter;

//   if (!converter.convert()) {
//     return -1;
//   }

//   converter.printOccupiedNodes();
//   converter.printQueryExamples();

//   // 如果你已经安装了 octovis，保留下面这行会自动弹窗显示。
//   converter.visualizeWithOctovis();

//   return 0;
// }
