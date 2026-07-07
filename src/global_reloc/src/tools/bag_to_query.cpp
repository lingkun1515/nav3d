// Offline bag -> query PCD extractor (dev/test tool).
// Reads a PointCloud2 topic from a rosbag2 over a time window [start, start+duration]
// (seconds, relative to the first message of the selected topic), merges all frames
// into one local LiDAR cloud and writes a binary PCD.
//
// Usage:
//   bag_to_query --bag <dir> --topic /utlidar/cloud --out q.pcd \
//                [--start 0.0] [--duration 2.0] [--voxel 0.0] [--stride 1]
//   bag_to_query --bag <dir> --imu-topic /utlidar/imu --dump-gravity \
//                [--start 0.0] [--duration 2.0]
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

namespace {

constexpr uint8_t PF_FLOAT32 = 7;
constexpr uint8_t PF_FLOAT64 = 8;

// Read one float per point field from a serialized PointCloud2 row.
float read_field(const uint8_t* base, const sensor_msgs::msg::PointField& f) {
  if (f.datatype == PF_FLOAT32) {
    float v = 0.0f;
    std::memcpy(&v, base + f.offset, sizeof(float));
    return v;
  }
  if (f.datatype == PF_FLOAT64) {
    double v = 0.0;
    std::memcpy(&v, base + f.offset, sizeof(double));
    return static_cast<float>(v);
  }
  return 0.0f;
}

}  // namespace

int main(int argc, char** argv) {
  std::string bag, topic = "/utlidar/cloud", out_path, imu_topic, starts_str, imu_gravity_str;
  double start_s = 0.0, duration_s = 2.0, voxel = 0.0;
  int stride = 1;
  bool dump_gravity = false;
  bool align_gravity = false;
  Eigen::Matrix3d R_align_ = Eigen::Matrix3d::Identity();

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](int& k) -> std::string { return (k + 1 < argc) ? argv[++k] : ""; };
    if (a == "--bag") bag = next(i);
    else if (a == "--topic") topic = next(i);
    else if (a == "--imu-topic") imu_topic = next(i);
    else if (a == "--out") out_path = next(i);
    else if (a == "--out-dir") out_path = next(i);
    else if (a == "--start") start_s = std::stod(next(i));
    else if (a == "--starts") starts_str = next(i);
    else if (a == "--duration") duration_s = std::stod(next(i));
    else if (a == "--voxel") voxel = std::stod(next(i));
    else if (a == "--stride") stride = std::stoi(next(i));
    else if (a == "--dump-gravity") dump_gravity = true;
    else if (a == "--align-gravity") align_gravity = true;
    else if (a == "--gravity") imu_gravity_str = next(i);
    else if (a == "--help" || a == "-h") {
      std::cout << "bag_to_query --bag <dir> --topic <t> --out q.pcd "
                   "[--start 0 --duration 2 --voxel 0 --stride 1]\n"
                   "  multi-window: --starts \"0 20 40 ...\" --out-dir <d> [--duration 2]\n"
                   "  --align-gravity: rotate points so IMU specific-force -> +Z (matches super_lio GT)\n"
                   "                   requires --imu-topic (or --gravity \"gx gy gz\")\n"
                   "bag_to_query --bag <dir> --imu-topic <t> --dump-gravity [--start 0 --duration 2]\n";
      return 0;
    }
  }
  if (bag.empty()) { std::cerr << "error: --bag required\n"; return 1; }

  // rclcpp::init registers the type support / serialization context.
  rclcpp::init(argc, argv);

  // Compute the gravity (specific-force) direction in the lidar frame for each
  // window so points can be rotated to a gravity-aligned frame (z-up), matching
  // the convention of super_lio's GT (which gravity-aligns via kf_align_gravity).
  // The IMU frame and lidar frame differ by extrinsic.lidar_imu; for the Mid360
  // mount here that rotation is identity (see reloc_run.yaml), so the IMU
  // specific-force direction applies directly to lidar points.
  bool have_grav = false;
  Eigen::Vector3d gravity_dir(0, 0, 1);
  if (align_gravity) {
    if (!imu_gravity_str.empty()) {
      std::stringstream ss(imu_gravity_str);
      double gx, gy, gz;
      if (ss >> gx >> gy >> gz) {
        gravity_dir = Eigen::Vector3d(gx, gy, gz).normalized();
        have_grav = true;
      }
    } else if (!imu_topic.empty()) {
      // Single shared gravity from the first window's IMU (robot mostly static
      // at start; good enough — the LiDAR mount pitch is constant).
      rosbag2_cpp::readers::SequentialReader ireader;
      ireader.open({bag, "sqlite3"}, {"cdr", "cdr"});
      ireader.set_filter(rosbag2_storage::StorageFilter{{imu_topic}});
      auto iser = rclcpp::Serialization<sensor_msgs::msg::Imu>();
      Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
      int64_t it0 = -1; int in = 0;
      while (ireader.has_next()) {
        auto m = ireader.read_next();
        if (it0 < 0) it0 = m->time_stamp;
        double rel = static_cast<double>(m->time_stamp - it0) / 1e9;
        if (rel > 2.0) break;  // first 2 s
        sensor_msgs::msg::Imu imu;
        rclcpp::SerializedMessage s(*m->serialized_data);
        iser.deserialize_message(&s, &imu);
        Eigen::Vector3d a(imu.linear_acceleration.x, imu.linear_acceleration.y,
                          imu.linear_acceleration.z);
        if (a.array().isFinite().all() && a.squaredNorm() > 1e-6) { acc_sum += a; ++in; }
      }
      if (in > 0) {
        gravity_dir = (acc_sum / static_cast<double>(in)).normalized();
        have_grav = true;
        std::cout << "GRAVITY_ALIGN n=" << in << " g=[" << gravity_dir.x() << " "
                  << gravity_dir.y() << " " << gravity_dir.z() << "]\n";
      }
    }
    if (!have_grav) {
      std::cerr << "--align-gravity needs --imu-topic or --gravity\n";
      return 1;
    }
    if (have_grav) {
      // Rotation that maps gravity_dir (current "up" in lidar frame) -> world +Z.
      Eigen::Quaterniond qg = Eigen::Quaterniond::FromTwoVectors(
          gravity_dir, Eigen::Vector3d::UnitZ());
      R_align_ = qg.toRotationMatrix();
    }
  }

  // --- IMU gravity dump ---
  if (dump_gravity) {
    if (imu_topic.empty()) { std::cerr << "--dump-gravity needs --imu-topic\n"; return 1; }
    rosbag2_cpp::readers::SequentialReader reader;
    reader.open({bag, "sqlite3"}, {"cdr", "cdr"});
    reader.set_filter(rosbag2_storage::StorageFilter{{imu_topic}});

    auto ser = rclcpp::Serialization<sensor_msgs::msg::Imu>();
    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_first;
    int64_t t0 = -1, n = 0;
    while (reader.has_next()) {
      auto m = reader.read_next();
      if (t0 < 0) t0 = m->time_stamp;
      double rel = static_cast<double>(m->time_stamp - t0) / 1e9;
      if (rel < start_s) continue;
      if (rel > start_s + duration_s) break;
      sensor_msgs::msg::Imu imu;
      rclcpp::SerializedMessage s(*m->serialized_data);
      ser.deserialize_message(&s, &imu);
      Eigen::Vector3d a(imu.linear_acceleration.x, imu.linear_acceleration.y,
                        imu.linear_acceleration.z);
      if (!a.array().isFinite().all() || a.squaredNorm() < 1e-6) continue;
      if (n == 0) acc_first = a;
      acc_sum += a;
      ++n;
    }
    if (n == 0) { std::cerr << "no IMU msgs in window\n"; return 2; }
    Eigen::Vector3d g = (acc_sum / static_cast<double>(n)).normalized();
    std::cout << "GRAVITY n=" << n << " g_lidar=[" << g.x() << " " << g.y() << " "
              << g.z() << "]\n";
    return 0;
  }

  // --- PointCloud extraction ---
  // Multi-window mode: --starts "0 20 40 ..." + --out-dir dumps q_<start>.pcd
  // for every window in a single sequential bag pass. Single-window mode
  // (legacy): --start/--out write one PCD.
  bool multi = !starts_str.empty();
  if (!multi && out_path.empty()) { std::cerr << "error: --out required\n"; return 1; }

  std::vector<double> starts;
  std::string out_dir = ".";
  if (multi) {
    std::stringstream ss(starts_str);
    double v;
    while (ss >> v) starts.push_back(v);
    if (starts.empty()) { std::cerr << "error: --starts parsed no values\n"; return 1; }
    if (!out_path.empty()) out_dir = out_path;  // reuse --out as dir in multi mode
  } else {
    starts.push_back(start_s);
  }

  struct Window { double start; double end; pcl::PointCloud<pcl::PointXYZI> merged; int frames; bool done; };
  std::vector<Window> wins;
  for (double s0 : starts) wins.push_back({s0, s0 + duration_s, {}, 0, false});
  double last_end = std::max_element(wins.begin(), wins.end(),
      [](const Window& a, const Window& b) { return a.end < b.end; })->end;

  rosbag2_cpp::readers::SequentialReader reader;
  reader.open({bag, "sqlite3"}, {"cdr", "cdr"});
  reader.set_filter(rosbag2_storage::StorageFilter{{topic}});

  auto ser = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
  int64_t t0 = -1;
  int frame_idx = 0;
  while (reader.has_next()) {
    auto m = reader.read_next();
    if (t0 < 0) t0 = m->time_stamp;
    double rel = static_cast<double>(m->time_stamp - t0) / 1e9;
    if (rel > last_end) break;
    bool any_active = false;
    for (auto& w : wins) if (rel >= w.start && rel < w.end) { any_active = true; break; }
    if (!any_active) continue;
    if ((frame_idx++ % std::max(1, stride)) != 0) continue;

    sensor_msgs::msg::PointCloud2 pc;
    rclcpp::SerializedMessage s(*m->serialized_data);
    ser.deserialize_message(&s, &pc);

    int xi = -1, yi = -1, zi = -1, ii = -1;
    for (size_t i = 0; i < pc.fields.size(); ++i) {
      const auto& f = pc.fields[i];
      if (f.name == "x") xi = static_cast<int>(i);
      else if (f.name == "y") yi = static_cast<int>(i);
      else if (f.name == "z") zi = static_cast<int>(i);
      else if (f.name == "intensity") ii = static_cast<int>(i);
    }
    if (xi < 0 || yi < 0 || zi < 0) continue;
    const auto& fx = pc.fields[xi];
    const auto& fy = pc.fields[yi];
    const auto& fz = pc.fields[zi];
    const auto* fi = (ii >= 0) ? &pc.fields[ii] : nullptr;
    const uint32_t step = pc.point_step;
    const auto& data = pc.data;

    for (auto& w : wins) {
      if (rel < w.start || rel >= w.end) continue;
      size_t before = w.merged.size();
      for (uint32_t r = 0; r < pc.height * pc.width; ++r) {
        const uint8_t* base = data.data() + r * step;
        pcl::PointXYZI p;
        p.x = read_field(base, fx);
        p.y = read_field(base, fy);
        p.z = read_field(base, fz);
        p.intensity = fi ? read_field(base, *fi) : 0.f;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (have_grav) {
          // Rotate so the IMU specific-force direction -> +Z (gravity-aligned,
          // z-up). This makes the query frame match super_lio's GT convention
          // (kf_align_gravity=true), so the two are directly comparable.
          Eigen::Vector3d v(p.x, p.y, p.z);
          Eigen::Vector3d vr = R_align_ * v;
          p.x = static_cast<float>(vr.x());
          p.y = static_cast<float>(vr.y());
          p.z = static_cast<float>(vr.z());
        }
        w.merged.push_back(p);
      }
      ++w.frames;
      if (!multi) {
        std::cout << "frame " << frame_idx - 1 << " rel=" << rel
                  << " +" << (w.merged.size() - before) << " pts (total " << w.merged.size() << ")\n";
      }
    }
  }

  // Write each window.
  for (auto& w : wins) {
    if (w.merged.empty()) {
      std::cerr << "no points in [" << w.start << "," << w.end << "]s\n";
      continue;
    }
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(w.merged, w.merged, idx);
    if (voxel > 0) {
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(w.merged.makeShared());
      vg.setLeafSize(voxel, voxel, voxel);
      pcl::PointCloud<pcl::PointXYZI> down;
      vg.filter(down);
      w.merged = down;
    }
    std::string path = multi
        ? (out_dir + "/q_" + std::to_string(static_cast<int>(w.start)) + ".pcd")
        : out_path;
    if (pcl::io::savePCDFileBinary(path, w.merged) < 0) {
      std::cerr << "failed to write " << path << "\n";
      continue;
    }
    std::cout << "wrote " << path << "  start=" << w.start
              << " frames=" << w.frames << " pts=" << w.merged.size() << "\n";
    // Absolute midpoint time (bag epoch, seconds) for GT alignment: rel window
    // [start, start+dur] -> abs = t0 + start + dur/2.
    double abs_mid = (t0 > 0)
        ? (static_cast<double>(t0) / 1e9 + w.start + duration_s / 2.0) : 0.0;
    std::cout << "ABSTIME " << w.start << " " << std::setprecision(9) << abs_mid
              << std::setprecision(6) << "\n";
  }
  return 0;
}
