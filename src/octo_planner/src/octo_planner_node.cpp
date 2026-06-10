#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"

#include "pcd2octomap_converter.h"
#include "global_planner.h"

using namespace std::chrono_literals;

class OctoPlannerNode : public rclcpp::Node
{
public:
  OctoPlannerNode() : Node("octo_planner_node")
  {
    declare_parameters();
    setup_pub_sub();

    std::string pcd_file = get_parameter("pcd_file").as_string();
    if (!pcd_file.empty()) {
      load_map(pcd_file);
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
    declare_parameter("octomap_publish_period_s", 1.0);
  }

  void setup_pub_sub()
  {
    auto qos_tl = rclcpp::QoS(1).transient_local().reliable();

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>("/octomap", qos_tl);
    occupied_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/octomap_occupied_markers", qos_tl);
    traversable_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/traversable_cells_markers", qos_tl);
    preblocked_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/preblocked_cells_markers", qos_tl);
    risk_cost_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/risk_cost_cells", qos_tl);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", qos_tl);

    start_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/start_point", qos_tl,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) { on_start(msg); });

    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", qos_tl,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) { on_goal(msg); });

    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", qos_tl,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_goal_pose(msg); });

    pcd_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/pcd_file_cmd", rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) { load_map(msg->data); });

    double period = get_parameter("octomap_publish_period_s").as_double();
    republish_timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      [this]() { republish_octomap(); });
  }

  void load_map(const std::string & pcd_file)
  {
    RCLCPP_INFO(get_logger(), "Loading PCD: %s", pcd_file.c_str());

    pcd2octomap::ConverterConfig conv_cfg;
    conv_cfg.input_pcd = pcd_file;
    conv_cfg.output_bt = "";
    conv_cfg.resolution = get_parameter("resolution").as_double();
    conv_cfg.min_points_per_voxel = get_parameter("min_points_per_voxel").as_int();
    conv_cfg.min_cluster_voxels = get_parameter("min_cluster_voxels").as_int();
    conv_cfg.save_to_file = false;

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

    RCLCPP_INFO(get_logger(), "OctoMap built. Resolution=%.3f, leaves=%zu",
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

    planner_ = std::make_unique<global_planner::GlobalPlanner>();
    planner_->configure(planner_cfg);
    planner_->setOctomap(octree_);

    map_ready_ = true;
    has_start_ = false;
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
    has_start_ = true;
    RCLCPP_INFO(get_logger(), "Start set: (%.2f, %.2f, %.2f)",
                start_point_.x, start_point_.y, start_point_.z);
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
    if (!map_ready_ || !has_start_ || !has_goal_) {
      if (!map_ready_) RCLCPP_WARN(get_logger(), "Map not ready.");
      if (!has_start_) RCLCPP_WARN(get_logger(), "Start not set.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Planning from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)",
                start_point_.x, start_point_.y, start_point_.z,
                goal_point_.x, goal_point_.y, goal_point_.z);

    auto t0 = now();
    planner_->makePlan(start_point_, goal_point_);

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

  void republish_octomap()
  {
    if (map_ready_) {
      publish_octomap();
    }
  }

  void publish_occupied_markers()
  {
    if (!octree_) return;
    std::string frame_id = get_parameter("frame_id").as_string();
    double res = octree_->getResolution();

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id;
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = res;
    marker.scale.y = res;
    marker.scale.z = res;
    marker.color.r = 0.95f;
    marker.color.g = 0.45f;
    marker.color.b = 0.15f;
    marker.color.a = 0.95f;
    marker.pose.orientation.w = 1.0;

    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (octree_->isNodeOccupied(*it)) {
        geometry_msgs::msg::Point p;
        p.x = it.getX();
        p.y = it.getY();
        p.z = it.getZ();
        marker.points.push_back(p);
      }
    }

    occupied_marker_pub_->publish(marker);
    RCLCPP_INFO(get_logger(), "Published %zu occupied voxels.", marker.points.size());
  }

  void publish_traversable_markers()
  {
    if (!planner_) return;
    std::string frame_id = get_parameter("frame_id").as_string();
    double res = planner_->getResolution();

    const auto & cells = planner_->getTraversableCells();

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id;
    marker.ns = "traversable_cells";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = res;
    marker.scale.y = res;
    marker.scale.z = res;
    marker.color.r = 0.20f;
    marker.color.g = 0.95f;
    marker.color.b = 0.55f;
    marker.color.a = 0.22f;
    marker.pose.orientation.w = 1.0;

    marker.points.reserve(cells.size());
    for (const auto & c : cells) {
      auto world = planner_->gridToWorldPublic(c);
      geometry_msgs::msg::Point p;
      p.x = world.x();
      p.y = world.y();
      p.z = world.z();
      marker.points.push_back(p);
    }

    traversable_marker_pub_->publish(marker);
    RCLCPP_INFO(get_logger(), "Published %zu traversable cells.", cells.size());
  }

  void publish_preblocked_markers()
  {
    if (!planner_) return;
    std::string frame_id = get_parameter("frame_id").as_string();
    double res = planner_->getResolution();

    const auto & cells = planner_->getPreblockedCells();

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id;
    marker.ns = "preblocked_cells";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = res;
    marker.scale.y = res;
    marker.scale.z = res;
    marker.color.r = 0.30f;
    marker.color.g = 0.51f;
    marker.color.b = 1.0f;
    marker.color.a = 0.92f;
    marker.pose.orientation.w = 1.0;

    marker.points.reserve(cells.size());
    for (const auto & c : cells) {
      auto world = planner_->gridToWorldPublic(c);
      geometry_msgs::msg::Point p;
      p.x = world.x();
      p.y = world.y();
      p.z = world.z();
      marker.points.push_back(p);
    }

    preblocked_marker_pub_->publish(marker);
    RCLCPP_INFO(get_logger(), "Published %zu preblocked cells.", cells.size());
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
    RCLCPP_INFO(get_logger(), "Published %zu risk cost cells.", costmap.size());
  }

  // Members
  std::unique_ptr<pcd2octomap::Pcd2OctomapConverter> converter_;
  std::unique_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;

  bool map_ready_ = false;
  bool has_start_ = false;
  bool has_goal_ = false;
  global_planner::PointPose start_point_{};
  global_planner::PointPose goal_point_{};

  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupied_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traversable_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr preblocked_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr risk_cost_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcd_cmd_sub_;

  rclcpp::TimerBase::SharedPtr republish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctoPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
