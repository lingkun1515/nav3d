#include "world_loader.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tinyxml2.h>

#include <octomap/OcTree.h>

namespace
{
struct Transform3
{
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
};

Transform3 compose(const Transform3 & a, const Transform3 & b)
{
  Transform3 out;
  out.R = a.R * b.R;
  out.t = a.R * b.t + a.t;
  return out;
}

Eigen::Vector3d apply(const Transform3 & tf, const Eigen::Vector3d & p)
{
  return tf.R * p + tf.t;
}

std::vector<double> parseDoubles(const std::string & s)
{
  std::istringstream iss(s);
  std::vector<double> vals;
  double v = 0.0;
  while (iss >> v) {
    vals.push_back(v);
  }
  return vals;
}

Transform3 parsePoseElement(const tinyxml2::XMLElement * pose_elem)
{
  Transform3 tf;
  if (!pose_elem || !pose_elem->GetText()) {
    return tf;
  }
  const auto vals = parseDoubles(pose_elem->GetText());
  if (vals.size() < 6) {
    return tf;
  }
  const double roll = vals[3];
  const double pitch = vals[4];
  const double yaw = vals[5];
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  tf.R = (rz * ry * rx).toRotationMatrix();
  tf.t = Eigen::Vector3d(vals[0], vals[1], vals[2]);
  return tf;
}

class WorldLoader
{
public:
  WorldLoader(double resolution,
              double xy_window_size_m,
              double ground_surface_max_thickness_m,
              bool enable_stair_step_surface_mode,
              double stair_step_max_height_m,
              double stair_step_max_depth_m,
              double stair_step_min_width_m)
  : half_xy_extent_m_(xy_window_size_m > 0.0 ? 0.5 * xy_window_size_m : -1.0),
    ground_surface_max_thickness_m_(ground_surface_max_thickness_m),
    enable_stair_step_surface_mode_(enable_stair_step_surface_mode),
    stair_step_max_height_m_(stair_step_max_height_m),
    stair_step_max_depth_m_(stair_step_max_depth_m),
    stair_step_min_width_m_(stair_step_min_width_m)
  {
    tree_ = std::make_shared<octomap::OcTree>(resolution);
  }

  std::shared_ptr<octomap::OcTree> generate(const std::string & world_file, int & shape_count)
  {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(world_file.c_str()) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("Failed to load world/sdf file: " + world_file);
    }

    const tinyxml2::XMLElement * sdf = doc.FirstChildElement("sdf");
    if (!sdf) {
      throw std::runtime_error("No <sdf> root in file: " + world_file);
    }
    const tinyxml2::XMLElement * world = sdf->FirstChildElement("world");
    if (!world) {
      throw std::runtime_error("No <world> in sdf file: " + world_file);
    }

    Transform3 world_tf;
    shape_count = 0;
    for (const tinyxml2::XMLElement * model = world->FirstChildElement("model");
         model; model = model->NextSiblingElement("model"))
    {
      const Transform3 model_tf = compose(world_tf,
        parsePoseElement(model->FirstChildElement("pose")));
      parseModel(model, model_tf, shape_count);
    }

    tree_->updateInnerOccupancy();
    return tree_;
  }

private:
  void parseModel(const tinyxml2::XMLElement * model,
                  const Transform3 & model_tf, int & shape_count)
  {
    for (const tinyxml2::XMLElement * link = model->FirstChildElement("link");
         link; link = link->NextSiblingElement("link"))
    {
      const Transform3 link_tf = compose(model_tf,
        parsePoseElement(link->FirstChildElement("pose")));
      for (const tinyxml2::XMLElement * collision = link->FirstChildElement("collision");
           collision; collision = collision->NextSiblingElement("collision"))
      {
        const Transform3 col_tf = compose(link_tf,
          parsePoseElement(collision->FirstChildElement("pose")));
        const tinyxml2::XMLElement * geom = collision->FirstChildElement("geometry");
        if (!geom) {
          continue;
        }
        if (const auto * box = geom->FirstChildElement("box")) {
          fillBox(col_tf, box);
          ++shape_count;
        } else if (const auto * cyl = geom->FirstChildElement("cylinder")) {
          fillCylinder(col_tf, cyl);
          ++shape_count;
        } else if (const auto * sph = geom->FirstChildElement("sphere")) {
          fillSphere(col_tf, sph);
          ++shape_count;
        } else if (const auto * plane = geom->FirstChildElement("plane")) {
          fillPlane(col_tf, plane);
          ++shape_count;
        }
      }
    }
  }

  void markPoint(const Eigen::Vector3d & p)
  {
    if (half_xy_extent_m_ > 0.0 &&
        (std::abs(p.x()) > half_xy_extent_m_ ||
         std::abs(p.y()) > half_xy_extent_m_))
    {
      return;
    }
    octomap::OcTreeKey key;
    const octomap::point3d q(
      static_cast<float>(p.x()),
      static_cast<float>(p.y()),
      static_cast<float>(p.z()));
    if (!tree_->coordToKeyChecked(q, key)) {
      return;
    }
    const octomap::point3d center = tree_->keyToCoord(key);
    tree_->updateNode(center, true);
  }

  void fillBox(const Transform3 & tf, const tinyxml2::XMLElement * box)
  {
    const auto * size_elem = box->FirstChildElement("size");
    if (!size_elem || !size_elem->GetText()) {
      return;
    }
    const auto vals = parseDoubles(size_elem->GetText());
    if (vals.size() < 3) {
      return;
    }
    const double sx = vals[0];
    const double sy = vals[1];
    const double sz = vals[2];
    const double r = tree_->getResolution();
    const double min_xy = std::min(sx, sy);
    const double max_xy = std::max(sx, sy);

    const Eigen::Vector3d local_z_in_world = tf.R * Eigen::Vector3d::UnitZ();
    const bool near_horizontal = std::abs(local_z_in_world.z()) > 0.9;
    const bool thin_ground_like = near_horizontal &&
      (sz <= ground_surface_max_thickness_m_);
    const bool stair_step_like =
      enable_stair_step_surface_mode_ && near_horizontal &&
      (sz <= stair_step_max_height_m_) &&
      (min_xy <= stair_step_max_depth_m_) &&
      (max_xy >= stair_step_min_width_m_);

    if (thin_ground_like || stair_step_like) {
      const double step = std::max(r * 0.5, 1e-3);
      const int nx = std::max(1, static_cast<int>(std::ceil(sx / step)));
      const int ny = std::max(1, static_cast<int>(std::ceil(sy / step)));
      for (int ix = 0; ix <= nx; ++ix) {
        const double x = -sx * 0.5 +
          (sx * static_cast<double>(ix) / static_cast<double>(nx));
        for (int iy = 0; iy <= ny; ++iy) {
          const double y = -sy * 0.5 +
            (sy * static_cast<double>(iy) / static_cast<double>(ny));
          markPoint(apply(tf, Eigen::Vector3d(x, y, sz * 0.5)));
        }
      }
      return;
    }

    const double min_dim = std::min({sx, sy, sz});
    const double step = (min_dim <= 4.0 * r) ? std::max(r * 0.5, 1e-3) : r;
    const int nx = std::max(1, static_cast<int>(std::ceil(sx / step)));
    const int ny = std::max(1, static_cast<int>(std::ceil(sy / step)));
    const int nz = std::max(1, static_cast<int>(std::ceil(sz / step)));

    for (int ix = 0; ix <= nx; ++ix) {
      const double x = -sx * 0.5 +
        (sx * static_cast<double>(ix) / static_cast<double>(nx));
      for (int iy = 0; iy <= ny; ++iy) {
        const double y = -sy * 0.5 +
          (sy * static_cast<double>(iy) / static_cast<double>(ny));
        for (int iz = 0; iz <= nz; ++iz) {
          const double z = -sz * 0.5 +
            (sz * static_cast<double>(iz) / static_cast<double>(nz));
          markPoint(apply(tf, Eigen::Vector3d(x, y, z)));
        }
      }
    }
  }

  void fillCylinder(const Transform3 & tf, const tinyxml2::XMLElement * cyl)
  {
    const auto * r_elem = cyl->FirstChildElement("radius");
    const auto * l_elem = cyl->FirstChildElement("length");
    if (!r_elem || !l_elem || !r_elem->GetText() || !l_elem->GetText()) {
      return;
    }
    const double radius = std::stod(r_elem->GetText());
    const double length = std::stod(l_elem->GetText());
    const double res = tree_->getResolution();
    for (double x = -radius; x <= radius; x += res) {
      for (double y = -radius; y <= radius; y += res) {
        if (x * x + y * y > radius * radius) {
          continue;
        }
        for (double z = -length * 0.5; z <= length * 0.5; z += res) {
          markPoint(apply(tf, Eigen::Vector3d(x, y, z)));
        }
      }
    }
  }

  void fillSphere(const Transform3 & tf, const tinyxml2::XMLElement * sph)
  {
    const auto * r_elem = sph->FirstChildElement("radius");
    if (!r_elem || !r_elem->GetText()) {
      return;
    }
    const double radius = std::stod(r_elem->GetText());
    const double res = tree_->getResolution();
    for (double x = -radius; x <= radius; x += res) {
      for (double y = -radius; y <= radius; y += res) {
        for (double z = -radius; z <= radius; z += res) {
          if (x * x + y * y + z * z > radius * radius) {
            continue;
          }
          markPoint(apply(tf, Eigen::Vector3d(x, y, z)));
        }
      }
    }
  }

  void fillPlane(const Transform3 & tf, const tinyxml2::XMLElement * plane)
  {
    const auto * size_elem = plane->FirstChildElement("size");
    if (!size_elem || !size_elem->GetText()) {
      return;
    }
    const auto vals = parseDoubles(size_elem->GetText());
    if (vals.size() < 2) {
      return;
    }
    const double sx = vals[0];
    const double sy = vals[1];
    const double r = tree_->getResolution();
    const double step = std::max(r * 0.5, 1e-3);
    const int nx = std::max(1, static_cast<int>(std::ceil(sx / step)));
    const int ny = std::max(1, static_cast<int>(std::ceil(sy / step)));
    for (int ix = 0; ix <= nx; ++ix) {
      const double x = -sx * 0.5 +
        (sx * static_cast<double>(ix) / static_cast<double>(nx));
      for (int iy = 0; iy <= ny; ++iy) {
        const double y = -sy * 0.5 +
          (sy * static_cast<double>(iy) / static_cast<double>(ny));
        markPoint(apply(tf, Eigen::Vector3d(x, y, 0.0)));
      }
    }
  }

  std::shared_ptr<octomap::OcTree> tree_;
  double half_xy_extent_m_;
  double ground_surface_max_thickness_m_;
  bool enable_stair_step_surface_mode_;
  double stair_step_max_height_m_;
  double stair_step_max_depth_m_;
  double stair_step_min_width_m_;
};

}  // namespace

std::shared_ptr<octomap::OcTree> loadWorldToOctomap(
    const std::string & world_file,
    double resolution,
    double xy_window_size_m,
    double ground_surface_max_thickness_m,
    bool enable_stair_step_surface_mode,
    double stair_step_max_height_m,
    double stair_step_max_depth_m,
    double stair_step_min_width_m)
{
  WorldLoader loader(resolution, xy_window_size_m,
                     ground_surface_max_thickness_m,
                     enable_stair_step_surface_mode,
                     stair_step_max_height_m,
                     stair_step_max_depth_m,
                     stair_step_min_width_m);
  int shape_count = 0;
  auto tree = loader.generate(world_file, shape_count);
  return tree;
}
