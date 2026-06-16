#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <sys/stat.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"

#include "pcd2octomap_converter.h"
#include "global_planner.h"
#include "world_loader.hpp"

using namespace std::chrono_literals;

class OctoPlannerNode : public rclcpp::Node
{
public:
  OctoPlannerNode() : Node("octo_planner_node")
  {
    declare_parameters();
    setup_pub_sub();

    std::string map_file = get_parameter("pcd_file").as_string();
    if (!map_file.empty()) {
      load_map_auto(map_file);
    } else {
      RCLCPP_INFO(get_logger(), "No pcd_file specified. Waiting for /pcd_file_cmd...");
    }
  }

private:
  void declare_parameters()
  {
    declare_parameter("pcd_file", "");
    declare_parameter("frame_id", "map");
    declare_parameter("resolution", 0.2);
    declare_parameter("min_points_per_voxel", 3);
    declare_parameter("min_cluster_voxels", 4);
    declare_parameter("enable_ground_infill", true);
    declare_parameter("ground_infill_neighbor_threshold", 3);
    declare_parameter("ground_infill_density_threshold", 0.02);
    declare_parameter("robot_radius", 0.25);
    declare_parameter("max_iterations", 500000);
    declare_parameter("snap_search_radius_cells", 8);
    declare_parameter("require_ground_support", true);
    declare_parameter("strict_direct_ground_support", true);
    declare_parameter("ground_support_xy_radius_cells", 1);
    declare_parameter("ground_support_depth_cells", 2);
    declare_parameter("enable_preblocked_costmap", true);
    declare_parameter("preblocked_costmap_radius_cells", 3);
    declare_parameter("preblocked_costmap_weight", 2.5);
    declare_parameter("lowest_traversable_only", false);
    declare_parameter("radical_infill_enabled", false);
    declare_parameter("radical_infill_radius_m", 1.0);
    declare_parameter("radical_infill_clearance_m", 1.0);
    declare_parameter("radical_infill_half_height_m", 0.1);
    declare_parameter("preblocked_hard_obstacle", true);
    declare_parameter("flatten_enabled", false);
    declare_parameter("flatten_window_cells", 5);
    declare_parameter("flatten_max_delta_cells", 1);
    declare_parameter("octomap_publish_period_s", 1.0);
    declare_parameter("auto_publish_enabled", false);
    declare_parameter("auto_save_bt", true);
    declare_parameter("world_xy_window_size_m", 24.0);
  }

  void setup_pub_sub()
  {
    auto qos_tl = rclcpp::QoS(1).transient_local().reliable();
    auto qos_tl_marker = rclcpp::QoS(50).transient_local().reliable();
    auto qos_sub = rclcpp::QoS(5).reliable();

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>("/octomap", qos_tl);
    occupied_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/octomap_occupied_markers", qos_tl_marker);
    traversable_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/traversable_cells_markers", qos_tl_marker);
    preblocked_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/preblocked_cells_markers", qos_tl_marker);
    risk_cost_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/risk_cost_cells", qos_tl);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", qos_tl);

    start_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/start_point", qos_sub,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) { on_start(msg); });

    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", qos_sub,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) { on_goal(msg); });

    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", qos_sub,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_goal_pose(msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(5).best_effort(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });

    pcd_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/pcd_file_cmd", rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) { load_map_auto(msg->data); });

    request_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "/request_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
              std_srvs::srv::Trigger::Response::SharedPtr res) {
        if (!map_ready_) {
          res->success = false;
          res->message = "Map not ready yet.";
          return;
        }
        republish_all();
        res->success = true;
        res->message = "Map data republished.";
      });

    if (get_parameter("auto_publish_enabled").as_bool()) {
      double period = get_parameter("octomap_publish_period_s").as_double();
      republish_timer_ = create_wall_timer(
        std::chrono::duration<double>(period),
        [this]() { republish_all(); });
    }
  }

  // ---- file utilities ----

  static bool file_exists(const std::string & path)
  {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
  }

  static bool is_newer_than(const std::string & a, const std::string & b)
  {
    struct stat sta, stb;
    if (stat(a.c_str(), &sta) != 0) return false;
    if (stat(b.c_str(), &stb) != 0) return true;
    return sta.st_mtime > stb.st_mtime;
  }

  static std::string replace_extension(const std::string & path, const std::string & new_ext)
  {
    namespace fs = std::filesystem;
    fs::path p(path);
    p.replace_extension(new_ext);
    return p.string();
  }

  // ---- format-aware loading ----

  void load_map_auto(const std::string & file_path)
  {
    if (file_path.empty()) return;

    std::string ext;
    auto dot = file_path.rfind('.');
    if (dot != std::string::npos) {
      ext = file_path.substr(dot);
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    }

    // For PCD/World sources, check if a .bt cache exists and is newer
    if (ext == ".pcd" || ext == ".world" || ext == ".sdf") {
      std::string cache_bt = replace_extension(file_path, ".bt");
      if (file_exists(cache_bt) && is_newer_than(cache_bt, file_path)) {
        RCLCPP_INFO(get_logger(), "Loading from .bt cache (newer than source): %s", cache_bt.c_str());
        load_bt_map(cache_bt);
        return;
      }
    }

    if (ext == ".pcd") {
      load_pcd_map(file_path);
    } else if (ext == ".bt") {
      load_bt_map(file_path);
    } else if (ext == ".ot") {
      load_ot_map(file_path);
    } else if (ext == ".world" || ext == ".sdf") {
      load_world_map(file_path);
    } else {
      RCLCPP_ERROR(get_logger(), "Unsupported map format: %s (supported: .pcd .bt .ot .world .sdf)",
                   file_path.c_str());
      return;
    }
  }

  void load_bt_map(const std::string & file_path)
  {
    RCLCPP_INFO(get_logger(), "Loading .bt: %s", file_path.c_str());
    try {
      octree_ = std::make_shared<octomap::OcTree>(file_path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load .bt file: %s", e.what());
      return;
    }
    if (octree_->size() == 0) {
      RCLCPP_ERROR(get_logger(), "Loaded .bt but tree is empty: %s", file_path.c_str());
      return;
    }
    configure_planner();
  }

  void load_ot_map(const std::string & file_path)
  {
    RCLCPP_INFO(get_logger(), "Loading .ot: %s", file_path.c_str());
    octomap::AbstractOcTree * raw = nullptr;
    try {
      raw = octomap::AbstractOcTree::read(file_path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load .ot file: %s", e.what());
      return;
    }
    if (!raw) {
      RCLCPP_ERROR(get_logger(), "Failed to read .ot file: %s", file_path.c_str());
      return;
    }
    octomap::OcTree * ot = dynamic_cast<octomap::OcTree *>(raw);
    if (!ot) {
      RCLCPP_ERROR(get_logger(), ".ot file is not an OcTree (got: %s)", raw->getTreeType().c_str());
      delete raw;
      return;
    }
    octree_.reset(ot);
    configure_planner();
  }

  void load_world_map(const std::string & file_path)
  {
    RCLCPP_INFO(get_logger(), "Loading .world/.sdf: %s", file_path.c_str());
    double resolution = get_parameter("resolution").as_double();
    double xy_win = get_parameter("world_xy_window_size_m").as_double();
    try {
      octree_ = loadWorldToOctomap(file_path, resolution, xy_win);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load world file: %s", e.what());
      return;
    }
    if (!octree_ || octree_->size() == 0) {
      RCLCPP_ERROR(get_logger(), "World file produced empty map: %s", file_path.c_str());
      return;
    }
    configure_planner();
    save_bt_cache(file_path);
  }

  void load_pcd_map(const std::string & pcd_file)
  {
    RCLCPP_INFO(get_logger(), "Loading PCD: %s", pcd_file.c_str());

    pcd2octomap::ConverterConfig conv_cfg;
    conv_cfg.input_pcd = pcd_file;
    conv_cfg.output_bt = "";
    conv_cfg.resolution = get_parameter("resolution").as_double();
    conv_cfg.min_points_per_voxel = get_parameter("min_points_per_voxel").as_int();
    conv_cfg.min_cluster_voxels = get_parameter("min_cluster_voxels").as_int();
    conv_cfg.save_to_file = false;
    conv_cfg.enable_ground_infill = get_parameter("enable_ground_infill").as_bool();
    conv_cfg.ground_infill_neighbor_threshold = get_parameter("ground_infill_neighbor_threshold").as_int();
    conv_cfg.ground_infill_density_threshold = get_parameter("ground_infill_density_threshold").as_double();

    converter_ = std::make_unique<pcd2octomap::Pcd2OctomapConverter>();
    converter_->configure(conv_cfg);

    if (!converter_->convert()) {
      RCLCPP_ERROR(get_logger(), "Failed to convert PCD file: %s", pcd_file.c_str());
      return;
    }

    octree_ = converter_->getOctomap();
    if (!octree_) {
      RCLCPP_ERROR(get_logger(), "OcTree is null after conversion.");
      return;
    }

    configure_planner();
    save_bt_cache(pcd_file);
  }

  void save_bt_cache(const std::string & source_file)
  {
    if (!get_parameter("auto_save_bt").as_bool()) return;
    if (!octree_) return;
    std::string bt_path = replace_extension(source_file, ".bt");
    if (octree_->writeBinary(bt_path)) {
      RCLCPP_INFO(get_logger(), "Saved .bt cache: %s", bt_path.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to save .bt cache: %s", bt_path.c_str());
    }
  }

  void configure_planner()
  {
    if (!octree_) return;

    RCLCPP_INFO(get_logger(), "OctoMap ready. Resolution=%.3f, leaves=%zu",
                octree_->getResolution(), octree_->getNumLeafNodes());

    global_planner::PlannerConfig planner_cfg;
    planner_cfg.robot_radius = get_parameter("robot_radius").as_double();
    planner_cfg.max_iterations = get_parameter("max_iterations").as_int();
    planner_cfg.snap_search_radius_cells = get_parameter("snap_search_radius_cells").as_int();
    planner_cfg.require_ground_support = get_parameter("require_ground_support").as_bool();
    planner_cfg.strict_direct_ground_support = get_parameter("strict_direct_ground_support").as_bool();
    planner_cfg.ground_support_xy_radius_cells = get_parameter("ground_support_xy_radius_cells").as_int();
    planner_cfg.ground_support_depth_cells = get_parameter("ground_support_depth_cells").as_int();
    planner_cfg.enable_preblocked_costmap = get_parameter("enable_preblocked_costmap").as_bool();
    planner_cfg.preblocked_costmap_radius_cells = get_parameter("preblocked_costmap_radius_cells").as_int();
    planner_cfg.preblocked_costmap_weight = get_parameter("preblocked_costmap_weight").as_double();
    planner_cfg.lowest_traversable_only = get_parameter("lowest_traversable_only").as_bool();
    planner_cfg.radical_infill_enabled = get_parameter("radical_infill_enabled").as_bool();
    planner_cfg.radical_infill_radius_m = get_parameter("radical_infill_radius_m").as_double();
    planner_cfg.radical_infill_clearance_m = get_parameter("radical_infill_clearance_m").as_double();
    planner_cfg.radical_infill_half_height_m = get_parameter("radical_infill_half_height_m").as_double();
    planner_cfg.preblocked_hard_obstacle = get_parameter("preblocked_hard_obstacle").as_bool();
    planner_cfg.flatten_enabled = get_parameter("flatten_enabled").as_bool();
    planner_cfg.flatten_window_cells = get_parameter("flatten_window_cells").as_int();
    planner_cfg.flatten_max_delta_cells = get_parameter("flatten_max_delta_cells").as_int();

    planner_ = std::make_unique<global_planner::GlobalPlanner>();
    planner_->configure(planner_cfg);
    planner_->setOctomap(octree_);

    map_ready_ = true;
    has_explicit_start_ = false;
    has_odom_ = false;
    has_goal_ = false;

    publish_octomap();
    publish_occupied_markers();
    publish_traversable_markers();
    publish_preblocked_markers();
    publish_risk_cost_cloud();

    RCLCPP_INFO(get_logger(), "Map loaded and published. Ready for planning.");
  }

  void on_start(geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    start_point_.x = msg->point.x;
    start_point_.y = msg->point.y;
    start_point_.z = msg->point.z;
    has_explicit_start_ = true;
    RCLCPP_INFO(get_logger(), "Start set (explicit): (%.2f, %.2f, %.2f)",
                start_point_.x, start_point_.y, start_point_.z);
  }

  void on_odom(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void on_goal(geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    goal_point_.x = msg->point.x;
    goal_point_.y = msg->point.y;
    goal_point_.z = msg->point.z;
    has_goal_ = true;
    RCLCPP_INFO(get_logger(), "Goal set: (%.2f, %.2f, %.2f)",
                goal_point_.x, goal_point_.y, goal_point_.z);
    try_plan();
  }

  void on_goal_pose(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_point_.x = msg->pose.position.x;
    goal_point_.y = msg->pose.position.y;
    goal_point_.z = msg->pose.position.z;
    has_goal_ = true;
    RCLCPP_INFO(get_logger(), "GoalPose set: (%.2f, %.2f, %.2f)",
                goal_point_.x, goal_point_.y, goal_point_.z);
    try_plan();
  }

  void try_plan()
  {
    // Resolve start point: explicit /start_point takes priority, fallback to odometry
    global_planner::PointPose start;
    if (has_explicit_start_) {
      start = start_point_;
    } else if (has_odom_) {
      const auto & p = latest_odom_.pose.pose.position;
      start = {p.x, p.y, p.z};
    } else {
      RCLCPP_WARN(get_logger(), "Start not set (no /start_point, no /odom).");
      return;
    }

    if (!map_ready_) {
      RCLCPP_WARN(get_logger(), "Map not ready.");
      return;
    }
    if (!has_goal_) {
      RCLCPP_WARN(get_logger(), "Goal not set.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Planning from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)%s",
                start.x, start.y, start.z,
                goal_point_.x, goal_point_.y, goal_point_.z,
                has_explicit_start_ ? "" : " [from odom]");

    auto t0 = now();
    planner_->makePlan(start, goal_point_);

    std::vector<global_planner::PointPose> results;
    planner_->getPlannerResults(results);
    auto dt = (now() - t0).seconds();

    if (results.empty()) {
      RCLCPP_WARN(get_logger(), "Planning failed. No path found. (%.3fs)", dt);
      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = now();
      empty_path.header.frame_id = get_parameter("frame_id").as_string();
      path_pub_->publish(empty_path);
      return;
    }

    RCLCPP_INFO(get_logger(), "Path found: %zu waypoints in %.3fs", results.size(), dt);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = get_parameter("frame_id").as_string();
    path_msg.poses.reserve(results.size());

    for (const auto & wp : results) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = wp.x;
      ps.pose.position.y = wp.y;
      ps.pose.position.z = wp.z;
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }

    path_pub_->publish(path_msg);
  }

  void publish_octomap()
  {
    if (!octree_) return;
    octomap_msgs::msg::Octomap msg;
    msg.header.stamp = now();
    msg.header.frame_id = get_parameter("frame_id").as_string();
    octomap_msgs::binaryMapToMsg(*octree_, msg);
    octomap_pub_->publish(msg);
  }

  void republish_all()
  {
    if (!map_ready_) return;
    publish_octomap();
    publish_occupied_markers();
    publish_traversable_markers();
    publish_preblocked_markers();
    publish_risk_cost_cloud();
  }

  static constexpr size_t MAX_POINTS_PER_MSG = 5000;

  void publish_marker_chunked(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub,
    const std::vector<geometry_msgs::msg::Point> & all_points,
    const std::string & ns, double res,
    float r, float g, float b, float a)
  {
    std::string frame_id = get_parameter("frame_id").as_string();
    int chunk_id = 0;

    for (size_t offset = 0; offset < all_points.size(); offset += MAX_POINTS_PER_MSG) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now();
      marker.header.frame_id = frame_id;
      marker.ns = ns;
      marker.id = chunk_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = res;
      marker.scale.y = res;
      marker.scale.z = res;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.pose.orientation.w = 1.0;

      size_t end = std::min(offset + MAX_POINTS_PER_MSG, all_points.size());
      marker.points.assign(all_points.begin() + offset, all_points.begin() + end);
      pub->publish(marker);
    }
  }

  void publish_occupied_markers()
  {
    if (!octree_) return;
    double res = octree_->getResolution();

    std::vector<geometry_msgs::msg::Point> points;
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (octree_->isNodeOccupied(*it)) {
        geometry_msgs::msg::Point p;
        p.x = it.getX();
        p.y = it.getY();
        p.z = it.getZ();
        points.push_back(p);
      }
    }

    publish_marker_chunked(occupied_marker_pub_, points,
                           "occupied_voxels", res, 0.95f, 0.45f, 0.15f, 0.95f);
  }

  void publish_traversable_markers()
  {
    if (!planner_) return;
    double res = planner_->getResolution();
    const auto & cells = planner_->getTraversableCells();

    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(cells.size());
    for (const auto & c : cells) {
      auto world = planner_->gridToWorldPublic(c);
      geometry_msgs::msg::Point p;
      p.x = world.x();
      p.y = world.y();
      p.z = world.z();
      points.push_back(p);
    }

    publish_marker_chunked(traversable_marker_pub_, points,
                           "traversable_cells", res, 0.20f, 0.95f, 0.55f, 0.22f);
  }

  void publish_preblocked_markers()
  {
    if (!planner_) return;
    double res = planner_->getResolution();
    const auto & cells = planner_->getPreblockedCells();

    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(cells.size());
    for (const auto & c : cells) {
      auto world = planner_->gridToWorldPublic(c);
      geometry_msgs::msg::Point p;
      p.x = world.x();
      p.y = world.y();
      p.z = world.z();
      points.push_back(p);
    }

    publish_marker_chunked(preblocked_marker_pub_, points,
                           "preblocked_cells", res, 0.30f, 0.51f, 1.0f, 0.92f);
  }

  void publish_risk_cost_cloud()
  {
    if (!planner_) return;
    std::string frame_id = get_parameter("frame_id").as_string();

    const auto & costmap = planner_->getPreblockedCostmap();
    if (costmap.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(costmap.size());
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(costmap.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");

    for (const auto & [idx, cost] : costmap) {
      auto world = planner_->gridToWorldPublic(idx);
      *iter_x = world.x();
      *iter_y = world.y();
      *iter_z = world.z();
      *iter_i = static_cast<float>(cost);
      ++iter_x; ++iter_y; ++iter_z; ++iter_i;
    }

    risk_cost_pub_->publish(cloud);
  }

  // Members
  std::unique_ptr<pcd2octomap::Pcd2OctomapConverter> converter_;
  std::unique_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;

  bool map_ready_ = false;
  bool has_explicit_start_ = false;
  bool has_odom_ = false;
  bool has_goal_ = false;
  global_planner::PointPose start_point_{};
  global_planner::PointPose goal_point_{};
  nav_msgs::msg::Odometry latest_odom_;

  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupied_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traversable_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr preblocked_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr risk_cost_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcd_cmd_sub_;

  rclcpp::TimerBase::SharedPtr republish_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_map_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctoPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
