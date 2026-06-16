#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace local_planner
{

constexpr double PI = 3.1415926;
constexpr int PATH_NUM = 343;
constexpr int GROUP_NUM = 7;
constexpr int GRID_VIXEL_NUM_X = 161;
constexpr int GRID_VIXEL_NUM_Y = 451;
constexpr int GRID_VIXEL_NUM = GRID_VIXEL_NUM_X * GRID_VIXEL_NUM_Y;

class LocalPlannerNode : public rclcpp::Node
{
public:
  LocalPlannerNode() : Node("localPlanner")
  {
    declare_parameters();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    load_path_files();
    setup_pub_sub();

    RCLCPP_INFO(get_logger(), "LocalPlanner initialization complete.");

    process_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() { process_loop(); });
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("pathFolder", "");
    declare_parameter("vehicleLength", 0.4);
    declare_parameter("vehicleWidth", 0.4);
    declare_parameter("sensorOffsetX", 0.0);
    declare_parameter("sensorOffsetY", 0.0);
    declare_parameter("vehicleLengthSlot", 0.05);
    declare_parameter("vehicleWidthMargin", 0.1);
    declare_parameter("marginYawRateRatio", 0.0);
    declare_parameter("twoWayDrive", true);
    declare_parameter("laserVoxelSize", 0.05);
    declare_parameter("terrainVoxelSize", 0.2);
    declare_parameter("useTerrainAnalysis", false);
    declare_parameter("checkObstacle", true);
    declare_parameter("checkRotObstacle", false);
    declare_parameter("adjacentRange", 3.5);
    declare_parameter("obstacleHeightThre", 0.2);
    declare_parameter("groundHeightThre", 0.1);
    declare_parameter("costHeightThre1", 0.15);
    declare_parameter("costHeightThre2", 0.1);
    declare_parameter("useCost", false);
    declare_parameter("slowPathNumThre", 5);
    declare_parameter("slowGroupNumThre", 1);
    declare_parameter("surPointThre", 2);
    declare_parameter("pointPerPathThre", 2);
    declare_parameter("minRelZ", -0.5);
    declare_parameter("maxRelZ", 0.25);
    declare_parameter("maxSpeed", 1.0);
    declare_parameter("dirWeight", 0.02);
    declare_parameter("dirThre", 90.0);
    declare_parameter("dirToVehicle", false);
    declare_parameter("pathScale", 1.0);
    declare_parameter("minPathScale", 0.75);
    declare_parameter("pathScaleStep", 0.25);
    declare_parameter("pathScaleBySpeed", true);
    declare_parameter("minPathRange", 1.0);
    declare_parameter("pathRangeStep", 0.5);
    declare_parameter("pathRangeBySpeed", true);
    declare_parameter("pathCropByGoal", true);
    declare_parameter("autonomyMode", false);
    declare_parameter("autonomySpeed", 1.0);
    declare_parameter("joyToSpeedDelay", 2.0);
    declare_parameter("joyToCheckObstacleDelay", 5.0);
    declare_parameter("freezeAng", 90.0);
    declare_parameter("freezeTime", 2.0);
    declare_parameter("omniDirGoalThre", 1.0);
    declare_parameter("goalClearRange", 0.5);
    declare_parameter("goalBehindRange", 0.8);
    declare_parameter("goalX", 0.0);
    declare_parameter("goalY", 0.0);

    declare_parameter<std::string>("global_frame_id", "odom");
    declare_parameter("use_planned_path", false);
    declare_parameter("use_laser_scan", false);
    declare_parameter("waypoint_lookahead", 2.5);
    declare_parameter("waypoint_tolerance", 0.5);
  }

  void setup_pub_sub()
  {
    auto qos = rclcpp::QoS(5);

    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odometry_callback(msg); });

    // Laser / obstacle cloud input
    if (use_laser_scan_) {
      sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { scan_callback(msg); });
    } else {
      sub_laser_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/registered_scan", qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { laser_cloud_callback(msg); });
    }

    sub_terrain_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/terrain_map", qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { terrain_cloud_callback(msg); });

    sub_joystick_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", qos,
      [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) { joystick_callback(msg); });

    // Goal input: /planned_path (with waypoint mgmt) or /way_point (direct)
    if (use_planned_path_) {
      sub_planned_path_ = create_subscription<nav_msgs::msg::Path>(
        "/planned_path", qos,
        [this](nav_msgs::msg::Path::ConstSharedPtr msg) { planned_path_callback(msg); });

      sub_start_nav_ = create_subscription<std_msgs::msg::Bool>(
        "/start_navigation", qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) { start_navigation_callback(msg); });
    } else {
      sub_goal_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/way_point", qos,
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) { goal_callback(msg); });
    }

    sub_speed_ = create_subscription<std_msgs::msg::Float32>(
      "/speed", qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr msg) { speed_callback(msg); });

    sub_boundary_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/navigation_boundary", qos,
      [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) { boundary_callback(msg); });

    sub_added_obstacles_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/added_obstacles", qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { added_obstacles_callback(msg); });

    sub_check_obstacle_ = create_subscription<std_msgs::msg::Bool>(
      "/check_obstacle", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) { check_obstacle_callback(msg); });

    pub_slow_down_ = create_publisher<std_msgs::msg::Int8>("/slow_down", qos);
    pub_sur_block_ = create_publisher<std_msgs::msg::Int8>("/surrounding_block", qos);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/path", qos);
    pub_free_paths_ = create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", rclcpp::QoS(2));
  }

  // ---- parameter accessors ----

  void read_params()
  {
    pathFolder_ = get_parameter("pathFolder").as_string();
    vehicleLength_ = get_parameter("vehicleLength").as_double();
    vehicleWidth_ = get_parameter("vehicleWidth").as_double();
    sensorOffsetX_ = get_parameter("sensorOffsetX").as_double();
    sensorOffsetY_ = get_parameter("sensorOffsetY").as_double();
    vehicleLengthSlot_ = get_parameter("vehicleLengthSlot").as_double();
    vehicleWidthMargin_ = get_parameter("vehicleWidthMargin").as_double();
    marginYawRateRatio_ = get_parameter("marginYawRateRatio").as_double();
    twoWayDrive_ = get_parameter("twoWayDrive").as_bool();
    laserVoxelSize_ = get_parameter("laserVoxelSize").as_double();
    terrainVoxelSize_ = get_parameter("terrainVoxelSize").as_double();
    useTerrainAnalysis_ = get_parameter("useTerrainAnalysis").as_bool();
    checkObstacle_ = get_parameter("checkObstacle").as_bool();
    checkRotObstacle_ = get_parameter("checkRotObstacle").as_bool();
    adjacentRange_ = get_parameter("adjacentRange").as_double();
    obstacleHeightThre_ = get_parameter("obstacleHeightThre").as_double();
    groundHeightThre_ = get_parameter("groundHeightThre").as_double();
    costHeightThre1_ = get_parameter("costHeightThre1").as_double();
    costHeightThre2_ = get_parameter("costHeightThre2").as_double();
    useCost_ = get_parameter("useCost").as_bool();
    slowPathNumThre_ = get_parameter("slowPathNumThre").as_int();
    slowGroupNumThre_ = get_parameter("slowGroupNumThre").as_int();
    surPointThre_ = get_parameter("surPointThre").as_int();
    pointPerPathThre_ = get_parameter("pointPerPathThre").as_int();
    minRelZ_ = get_parameter("minRelZ").as_double();
    maxRelZ_ = get_parameter("maxRelZ").as_double();
    maxSpeed_ = get_parameter("maxSpeed").as_double();
    dirWeight_ = get_parameter("dirWeight").as_double();
    dirThre_ = get_parameter("dirThre").as_double();
    dirToVehicle_ = get_parameter("dirToVehicle").as_bool();
    pathScale_ = get_parameter("pathScale").as_double();
    minPathScale_ = get_parameter("minPathScale").as_double();
    pathScaleStep_ = get_parameter("pathScaleStep").as_double();
    pathScaleBySpeed_ = get_parameter("pathScaleBySpeed").as_bool();
    minPathRange_ = get_parameter("minPathRange").as_double();
    pathRangeStep_ = get_parameter("pathRangeStep").as_double();
    pathRangeBySpeed_ = get_parameter("pathRangeBySpeed").as_bool();
    pathCropByGoal_ = get_parameter("pathCropByGoal").as_bool();
    autonomyMode_ = get_parameter("autonomyMode").as_bool();
    autonomySpeed_ = get_parameter("autonomySpeed").as_double();
    joyToSpeedDelay_ = get_parameter("joyToSpeedDelay").as_double();
    joyToCheckObstacleDelay_ = get_parameter("joyToCheckObstacleDelay").as_double();
    freezeAng_ = get_parameter("freezeAng").as_double();
    freezeTime_ = get_parameter("freezeTime").as_double();
    omniDirGoalThre_ = get_parameter("omniDirGoalThre").as_double();
    goalClearRange_ = get_parameter("goalClearRange").as_double();
    goalBehindRange_ = get_parameter("goalBehindRange").as_double();
    global_frame_id_ = get_parameter("global_frame_id").as_string();
    use_planned_path_ = get_parameter("use_planned_path").as_bool();
    use_laser_scan_ = get_parameter("use_laser_scan").as_bool();
    waypoint_lookahead_ = get_parameter("waypoint_lookahead").as_double();
    waypoint_tolerance_ = get_parameter("waypoint_tolerance").as_double();

    // Init autonomy speed
    if (autonomyMode_) {
      joySpeed_ = autonomySpeed_ / maxSpeed_;
      if (joySpeed_ < 0) joySpeed_ = 0;
      else if (joySpeed_ > 1.0f) joySpeed_ = 1.0f;
    }
  }

  // ---- PLY file loading ----

  int read_ply_header(FILE * fp)
  {
    char str[128];
    int val, pointNum = 0;
    std::string strCur, strLast;
    while (strCur != "end_header") {
      val = fscanf(fp, "%s", str);
      if (val != 1) {
        RCLCPP_ERROR(get_logger(), "Error reading PLY header.");
        return 0;
      }
      strLast = strCur;
      strCur = std::string(str);
      if (strCur == "vertex" && strLast == "element") {
        val = fscanf(fp, "%d", &pointNum);
        if (val != 1) {
          RCLCPP_ERROR(get_logger(), "Error reading vertex count.");
          return 0;
        }
      }
    }
    return pointNum;
  }

  void load_start_paths()
  {
    std::string fileName = pathFolder_ + "/startPaths.ply";
    FILE * fp = fopen(fileName.c_str(), "r");
    if (!fp) {
      RCLCPP_ERROR(get_logger(), "Cannot read %s", fileName.c_str());
      return;
    }

    int pointNum = read_ply_header(fp);
    for (int i = 0; i < pointNum; i++) {
      pcl::PointXYZ point;
      int groupID;
      if (fscanf(fp, "%f %f %f %d", &point.x, &point.y, &point.z, &groupID) != 4) {
        RCLCPP_ERROR(get_logger(), "Error reading startPaths.ply");
        break;
      }
      if (groupID >= 0 && groupID < GROUP_NUM) {
        startPaths_[groupID]->push_back(point);
      }
    }
    fclose(fp);
  }

  void load_paths()
  {
    std::string fileName = pathFolder_ + "/paths.ply";
    FILE * fp = fopen(fileName.c_str(), "r");
    if (!fp) {
      RCLCPP_ERROR(get_logger(), "Cannot read %s", fileName.c_str());
      return;
    }

    int pointNum = read_ply_header(fp);
    int pointSkipNum = 30;
    int pointSkipCount = 0;

    for (int i = 0; i < pointNum; i++) {
      pcl::PointXYZI point;
      int pathID;
      if (fscanf(fp, "%f %f %f %d %f", &point.x, &point.y, &point.z, &pathID, &point.intensity) != 5) {
        RCLCPP_ERROR(get_logger(), "Error reading paths.ply");
        break;
      }
      if (pathID >= 0 && pathID < PATH_NUM) {
        pointSkipCount++;
        if (pointSkipCount > pointSkipNum) {
          paths_[pathID]->push_back(point);
          pointSkipCount = 0;
        }
      }
    }
    fclose(fp);
  }

  void load_path_list()
  {
    std::string fileName = pathFolder_ + "/pathList.ply";
    FILE * fp = fopen(fileName.c_str(), "r");
    if (!fp) {
      RCLCPP_ERROR(get_logger(), "Cannot read %s", fileName.c_str());
      return;
    }

    if (PATH_NUM != read_ply_header(fp)) {
      RCLCPP_ERROR(get_logger(), "Incorrect path number in pathList.ply");
      return;
    }

    for (int i = 0; i < PATH_NUM; i++) {
      int pathID, groupID;
      float endX, endY, endZ;
      if (fscanf(fp, "%f %f %f %d %d", &endX, &endY, &endZ, &pathID, &groupID) != 5) {
        RCLCPP_ERROR(get_logger(), "Error reading pathList.ply");
        break;
      }
      if (pathID >= 0 && pathID < PATH_NUM && groupID >= 0 && groupID < GROUP_NUM) {
        pathList_[pathID] = groupID;
        endDirPathList_[pathID] = 2.0f * std::atan2(endY, endX) * 180.0f / PI;
      }
    }
    fclose(fp);
  }

  void load_correspondences()
  {
    std::string fileName = pathFolder_ + "/correspondences.txt";
    FILE * fp = fopen(fileName.c_str(), "r");
    if (!fp) {
      RCLCPP_ERROR(get_logger(), "Cannot read %s", fileName.c_str());
      return;
    }

    for (int i = 0; i < GRID_VIXEL_NUM; i++) {
      int gridVoxelID;
      if (fscanf(fp, "%d", &gridVoxelID) != 1) {
        RCLCPP_ERROR(get_logger(), "Error reading correspondences.txt");
        break;
      }
      correspondences_[i].clear();
      while (true) {
        int pathID;
        if (fscanf(fp, "%d", &pathID) != 1) {
          RCLCPP_ERROR(get_logger(), "Error reading correspondences.txt");
          break;
        }
        if (pathID == -1) break;
        if (gridVoxelID >= 0 && gridVoxelID < GRID_VIXEL_NUM && pathID >= 0 && pathID < PATH_NUM) {
          correspondences_[gridVoxelID].push_back(pathID);
        }
      }
    }
    fclose(fp);
  }

  void load_path_files()
  {
    read_params();

    for (int i = 0; i < GROUP_NUM; i++) {
      startPaths_[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    for (int i = 0; i < PATH_NUM; i++) {
      paths_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    laserDwzFilter_.setLeafSize(laserVoxelSize_, laserVoxelSize_, laserVoxelSize_);
    terrainDwzFilter_.setLeafSize(terrainVoxelSize_, terrainVoxelSize_, terrainVoxelSize_);

    load_start_paths();
    load_paths();
    load_path_list();
    load_correspondences();

    RCLCPP_INFO(get_logger(), "Path files loaded from: %s", pathFolder_.c_str());
  }

  // ---- callbacks ----

  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    odomTime_ = rclcpp::Time(odom->header.stamp).seconds();
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll_ = roll;
    vehiclePitch_ = pitch;
    vehicleYaw_ = yaw;
    vehicleYawRate_ = odom->twist.twist.angular.z;
    vehicleX_ = odom->pose.pose.position.x - std::cos(yaw) * sensorOffsetX_ + std::sin(yaw) * sensorOffsetY_;
    vehicleY_ = odom->pose.pose.position.y - std::sin(yaw) * sensorOffsetX_ - std::cos(yaw) * sensorOffsetY_;
    vehicleZ_ = odom->pose.pose.position.z;
  }

  void laser_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
  {
    if (useTerrainAnalysis_) return;

    // TF to global frame if needed
    if (!laserCloud2->header.frame_id.empty() &&
        laserCloud2->header.frame_id != global_frame_id_) {
      try {
        auto transform = tf_buffer_->lookupTransform(
          global_frame_id_, laserCloud2->header.frame_id, laserCloud2->header.stamp,
          rclcpp::Duration::from_seconds(0.1));
        sensor_msgs::msg::PointCloud2 pc2_transformed;
        tf2::doTransform(*laserCloud2, pc2_transformed, transform);
        pc2_transformed.header.frame_id = global_frame_id_;
        laserCloud_->clear();
        pcl::fromROSMsg(pc2_transformed, *laserCloud_);
      } catch (tf2::TransformException & e) {
        RCLCPP_DEBUG(get_logger(), "laserCloud TF failed: %s", e.what());
        return;
      }
    } else {
      laserCloud_->clear();
      pcl::fromROSMsg(*laserCloud2, *laserCloud_);
    }

    pcl::PointXYZI point;
    laserCloudCrop_->clear();
    for (const auto & pt : laserCloud_->points) {
      float dis = std::sqrt(
        (pt.x - vehicleX_) * (pt.x - vehicleX_) +
        (pt.y - vehicleY_) * (pt.y - vehicleY_));
      if (dis < adjacentRange_) {
        laserCloudCrop_->push_back(pt);
      }
    }

    laserCloudDwz_->clear();
    laserDwzFilter_.setInputCloud(laserCloudCrop_);
    laserDwzFilter_.filter(*laserCloudDwz_);

    newLaserCloud_ = true;
  }

  void terrain_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
  {
    if (!useTerrainAnalysis_) return;

    terrainCloud_->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud_);

    pcl::PointXYZI point;
    terrainCloudCrop_->clear();
    for (const auto & pt : terrainCloud_->points) {
      float dis = std::sqrt(
        (pt.x - vehicleX_) * (pt.x - vehicleX_) +
        (pt.y - vehicleY_) * (pt.y - vehicleY_));
      if (dis < adjacentRange_ &&
          (pt.intensity > obstacleHeightThre_ ||
           (pt.intensity > groundHeightThre_ && useCost_))) {
        terrainCloudCrop_->push_back(pt);
      }
    }

    terrainCloudDwz_->clear();
    terrainDwzFilter_.setInputCloud(terrainCloudCrop_);
    terrainDwzFilter_.filter(*terrainCloudDwz_);

    newTerrainCloud_ = true;
  }

  void joystick_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
  {
    joyTime_ = now().seconds();
    joySpeedRaw_ = std::sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
    joySpeed_ = joySpeedRaw_;
    if (joySpeed_ > 1.0f) joySpeed_ = 1.0f;
    if (joy->axes[4] == 0) joySpeed_ = 0;

    if (joySpeed_ > 0) {
      joyDir_ = std::atan2(joy->axes[3], joy->axes[4]) * 180.0f / PI;
      if (joy->axes[4] < 0) joyDir_ *= -1;
    }

    if (joy->axes[4] < 0 && !twoWayDrive_) joySpeed_ = 0;

    autonomyMode_ = (joy->axes[2] <= -0.1);
    checkObstacle_ = (joy->axes[5] <= -0.1);
  }

  void goal_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
  {
    goalX_ = goal->point.x;
    goalY_ = goal->point.y;
    has_goal_ = true;
  }

  // ---- LaserScan → PointCloud2 conversion with TF ----

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    if (useTerrainAnalysis_) return;

    int num_points = static_cast<int>(scan->ranges.size());
    if (num_points == 0) return;

    // Build PointCloud2 in scan frame (x forward, y left, z up)
    sensor_msgs::msg::PointCloud2 pc2;
    pc2.header = scan->header;
    pc2.height = 1;
    pc2.is_dense = false;

    pc2.fields.resize(4);
    pc2.fields[0].name = "x"; pc2.fields[0].offset = 0;
    pc2.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; pc2.fields[0].count = 1;
    pc2.fields[1].name = "y"; pc2.fields[1].offset = 4;
    pc2.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; pc2.fields[1].count = 1;
    pc2.fields[2].name = "z"; pc2.fields[2].offset = 8;
    pc2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; pc2.fields[2].count = 1;
    pc2.fields[3].name = "intensity"; pc2.fields[3].offset = 12;
    pc2.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32; pc2.fields[3].count = 1;
    pc2.point_step = 16;
    pc2.row_step = pc2.point_step * num_points;
    pc2.width = num_points;

    std::vector<float> data(num_points * 4, 0.0f);
    float angle = scan->angle_min;
    int valid = 0;
    for (int i = 0; i < num_points; i++) {
      float r = scan->ranges[i];
      if (scan->range_min < r && r < scan->range_max) {
        data[valid * 4 + 0] = r * std::cos(angle);
        data[valid * 4 + 1] = r * std::sin(angle);
        data[valid * 4 + 2] = 0.0f;
        data[valid * 4 + 3] = 1.0f;  // obstacle intensity
        valid++;
      }
      angle += scan->angle_increment;
    }
    pc2.width = valid;
    pc2.row_step = pc2.point_step * valid;
    pc2.data.resize(valid * 16);
    std::memcpy(pc2.data.data(), data.data(), valid * 16);

    if (valid == 0) return;

    // TF to global frame
    std::string scan_frame = scan->header.frame_id;
    if (scan_frame.empty()) scan_frame = "lidar_link";

    try {
      auto transform = tf_buffer_->lookupTransform(
        global_frame_id_, scan_frame, scan->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
      sensor_msgs::msg::PointCloud2 pc2_transformed;
      tf2::doTransform(pc2, pc2_transformed, transform);
      pc2_transformed.header.frame_id = global_frame_id_;
      auto pc2_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(std::move(pc2_transformed));
      laser_cloud_callback(pc2_ptr);
    } catch (tf2::TransformException & e) {
      RCLCPP_DEBUG(get_logger(), "scan TF failed: %s", e.what());
    }
  }

  // ---- /planned_path with waypoint management ----

  void planned_path_callback(const nav_msgs::msg::Path::ConstSharedPtr path)
  {
    if (path->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty planned path");
      return;
    }
    planned_waypoints_.clear();
    for (const auto & pose : path->poses) {
      planned_waypoints_.emplace_back(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);
    }
    current_wp_idx_ = 0;
    if (autonomyMode_) {
      navigating_ = true;
    }
    RCLCPP_INFO(get_logger(), "Received planned path with %zu waypoints", planned_waypoints_.size());
  }

  void start_navigation_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (msg->data && !planned_waypoints_.empty()) {
      navigating_ = true;
      current_wp_idx_ = 0;
      RCLCPP_INFO(get_logger(), "Navigation started");
    }
  }

  void speed_callback(const std_msgs::msg::Float32::ConstSharedPtr speed)
  {
    double speedTime = now().seconds();
    if (autonomyMode_ && speedTime - joyTime_ > joyToSpeedDelay_ && joySpeedRaw_ == 0) {
      joySpeed_ = speed->data / maxSpeed_;
      if (joySpeed_ < 0) joySpeed_ = 0;
      else if (joySpeed_ > 1.0f) joySpeed_ = 1.0f;
    }
  }

  void boundary_callback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr boundary)
  {
    boundaryCloud_->clear();
    int boundarySize = boundary->polygon.points.size();
    if (boundarySize < 1) return;

    pcl::PointXYZI point, point1, point2;
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;

    for (int i = 0; i < boundarySize; i++) {
      point1 = point2;
      point2.x = boundary->polygon.points[i].x;
      point2.y = boundary->polygon.points[i].y;
      point2.z = boundary->polygon.points[i].z;

      if (point1.z == point2.z) {
        float disX = point1.x - point2.x;
        float disY = point1.y - point2.y;
        float dis = std::sqrt(disX * disX + disY * disY);

        int pNum = static_cast<int>(dis / terrainVoxelSize_) + 1;
        for (int pid = 0; pid < pNum; pid++) {
          float ratio = static_cast<float>(pid) / static_cast<float>(pNum);
          point.x = ratio * point1.x + (1.0f - ratio) * point2.x;
          point.y = ratio * point1.y + (1.0f - ratio) * point2.y;
          point.z = 0;
          point.intensity = 100.0f;

          for (int j = 0; j < pointPerPathThre_; j++) {
            boundaryCloud_->push_back(point);
          }
        }
      }
    }
  }

  void added_obstacles_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr addedObs2)
  {
    addedObstacles_->clear();
    pcl::fromROSMsg(*addedObs2, *addedObstacles_);
    for (auto & pt : addedObstacles_->points) {
      pt.intensity = 200.0f;
    }
  }

  void check_obstacle_callback(const std_msgs::msg::Bool::ConstSharedPtr checkObs)
  {
    double checkObsTime = now().seconds();
    if (autonomyMode_ && checkObsTime - joyTime_ > joyToCheckObstacleDelay_) {
      checkObstacle_ = checkObs->data;
    }
  }

  // ---- main processing loop (100 Hz) ----

  void process_loop()
  {
    if (!newLaserCloud_ && !newTerrainCloud_) return;

    // Gather obstacle points
    if (newLaserCloud_) {
      newLaserCloud_ = false;
      plannerCloud_->clear();
      *plannerCloud_ = *laserCloudDwz_;
    }

    if (newTerrainCloud_) {
      newTerrainCloud_ = false;
      plannerCloud_->clear();
      *plannerCloud_ = *terrainCloudDwz_;
    }

    float sinYaw = std::sin(vehicleYaw_);
    float cosYaw = std::cos(vehicleYaw_);

    // Transform points to vehicle frame
    plannerCloudCrop_->clear();
    for (const auto & pt : plannerCloud_->points) {
      float px = pt.x - vehicleX_;
      float py = pt.y - vehicleY_;
      float pz = pt.z - vehicleZ_;

      pcl::PointXYZI point;
      point.x = px * cosYaw + py * sinYaw;
      point.y = -px * sinYaw + py * cosYaw;
      point.z = pz;
      point.intensity = pt.intensity;

      float dis = std::sqrt(point.x * point.x + point.y * point.y);
      if (dis < adjacentRange_ &&
          ((point.z > minRelZ_ && point.z < maxRelZ_) || useTerrainAnalysis_)) {
        plannerCloudCrop_->push_back(point);
      }
    }

    // Add boundary points
    for (const auto & pt : boundaryCloud_->points) {
      pcl::PointXYZI point;
      point.x = (pt.x - vehicleX_) * cosYaw + (pt.y - vehicleY_) * sinYaw;
      point.y = -(pt.x - vehicleX_) * sinYaw + (pt.y - vehicleY_) * cosYaw;
      point.z = pt.z;
      point.intensity = pt.intensity;

      float dis = std::sqrt(point.x * point.x + point.y * point.y);
      if (dis < adjacentRange_) {
        plannerCloudCrop_->push_back(point);
      }
    }

    // Add manual obstacles
    for (const auto & pt : addedObstacles_->points) {
      pcl::PointXYZI point;
      point.x = (pt.x - vehicleX_) * cosYaw + (pt.y - vehicleY_) * sinYaw;
      point.y = -(pt.x - vehicleX_) * sinYaw + (pt.y - vehicleY_) * cosYaw;
      point.z = pt.z;
      point.intensity = pt.intensity;

      float dis = std::sqrt(point.x * point.x + point.y * point.y);
      if (dis < adjacentRange_) {
        plannerCloudCrop_->push_back(point);
      }
    }

    // --- Surrounding block detection ---
    int brCount = 0, blCount = 0, frCount = 0, flCount = 0, rCount = 0, lCount = 0;
    for (const auto & pt : plannerCloudCrop_->points) {
      float x = pt.x;
      float y = pt.y;
      float h = pt.intensity;

      float margin = std::abs(marginYawRateRatio_ * x * vehicleYawRate_);
      float marginCW = 0, marginCCW = 0;
      if (vehicleYawRate_ < 0) marginCW = margin;
      else marginCCW = margin;

      if (h > obstacleHeightThre_ || !useTerrainAnalysis_) {
        if (x > -vehicleLength_ / 2.0 && x < -vehicleLengthSlot_ &&
            y > -vehicleWidth_ / 2.0 - vehicleWidthMargin_ - marginCCW &&
            y < -vehicleWidth_ / 2.0) brCount++;
        if (x > -vehicleLength_ / 2.0 && x < -vehicleLengthSlot_ &&
            y > vehicleWidth_ / 2.0 &&
            y < vehicleWidth_ / 2.0 + vehicleWidthMargin_ + marginCW) blCount++;
        if (x > vehicleLengthSlot_ && x < vehicleLength_ / 2.0 &&
            y > -vehicleWidth_ / 2.0 - vehicleWidthMargin_ - marginCW &&
            y < -vehicleWidth_ / 2.0) frCount++;
        if (x > vehicleLengthSlot_ && x < vehicleLength_ / 2.0 &&
            y > vehicleWidth_ / 2.0 &&
            y < vehicleWidth_ / 2.0 + vehicleWidthMargin_ + marginCCW) flCount++;
        if (x > -vehicleLength_ / 2.0 && x < vehicleLength_ / 2.0 &&
            y > -vehicleWidth_ / 2.0 - vehicleWidthMargin_ &&
            y < -vehicleWidth_ / 2.0) rCount++;
        if (x > -vehicleLength_ / 2.0 && x < vehicleLength_ / 2.0 &&
            y > vehicleWidth_ / 2.0 &&
            y < vehicleWidth_ / 2.0 + vehicleWidthMargin_) lCount++;
      }
    }

    std_msgs::msg::Int8 block_msg;
    block_msg.data = 0;
    if (brCount >= surPointThre_) block_msg.data += 1;
    if (blCount >= surPointThre_) block_msg.data += 2;
    if (frCount >= surPointThre_) block_msg.data += 4;
    if (flCount >= surPointThre_) block_msg.data += 8;
    if (rCount >= surPointThre_) block_msg.data += 16;
    if (lCount >= surPointThre_) block_msg.data += 32;
    pub_sur_block_->publish(block_msg);

    // --- Waypoint management from /planned_path ---
    if (use_planned_path_ && navigating_ && !planned_waypoints_.empty()) {
      // Find lookahead waypoint
      int target_idx = static_cast<int>(current_wp_idx_);
      for (size_t i = current_wp_idx_; i < planned_waypoints_.size(); i++) {
        double wx = std::get<0>(planned_waypoints_[i]);
        double wy = std::get<1>(planned_waypoints_[i]);
        double dist = std::hypot(wx - vehicleX_, wy - vehicleY_);
        if (dist > waypoint_lookahead_) {
          target_idx = static_cast<int>(i);
          break;
        }
        target_idx = static_cast<int>(i);
      }
      goalX_ = std::get<0>(planned_waypoints_[target_idx]);
      goalY_ = std::get<1>(planned_waypoints_[target_idx]);
      has_goal_ = true;

      // Advance reached waypoints
      while (current_wp_idx_ < planned_waypoints_.size() - 1) {
        double wx = std::get<0>(planned_waypoints_[current_wp_idx_]);
        double wy = std::get<1>(planned_waypoints_[current_wp_idx_]);
        double dist = std::hypot(wx - vehicleX_, wy - vehicleY_);
        if (dist < waypoint_tolerance_) {
          current_wp_idx_++;
        } else {
          break;
        }
      }

      // Check final goal reached
      if (current_wp_idx_ >= planned_waypoints_.size() - 1) {
        double wx = std::get<0>(planned_waypoints_.back());
        double wy = std::get<1>(planned_waypoints_.back());
        double dist = std::hypot(wx - vehicleX_, wy - vehicleY_);
        if (dist < waypoint_tolerance_) {
          navigating_ = false;
          has_goal_ = false;
          RCLCPP_INFO(get_logger(), "Navigation complete - goal reached");
        }
      }
    }

    // --- Determine joyDir and goal ---
    float pathRange = adjacentRange_;
    if (pathRangeBySpeed_) pathRange = adjacentRange_ * joySpeed_;
    if (pathRange < minPathRange_) pathRange = minPathRange_;
    float relativeGoalDis = adjacentRange_;

    int preSelectedGroupID = -1;
    float joyDir = joyDir_;

    if (autonomyMode_) {
      if (!has_goal_) {
        relativeGoalDis = 0;
        joyDir = 0;
      } else {
        float relativeGoalX = ((goalX_ - vehicleX_) * cosYaw + (goalY_ - vehicleY_) * sinYaw);
        float relativeGoalY = (-(goalX_ - vehicleX_) * sinYaw + (goalY_ - vehicleY_) * cosYaw);

        relativeGoalDis = std::sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        joyDir = std::atan2(relativeGoalY, relativeGoalX) * 180.0f / PI;
      }

      if (std::abs(joyDir) > freezeAng_ && relativeGoalDis < goalBehindRange_) {
        relativeGoalDis = 0;
        joyDir = 0;
      }

      if (std::abs(joyDir) > freezeAng_ && freezeStatus_ == 0) {
        freezeStartTime_ = odomTime_;
        freezeStatus_ = 1;
      } else if (odomTime_ - freezeStartTime_ > freezeTime_ && freezeStatus_ == 1) {
        freezeStatus_ = 2;
      } else if (std::abs(joyDir) <= freezeAng_ && freezeStatus_ == 2) {
        freezeStatus_ = 0;
      }

      if (!twoWayDrive_) {
        if (joyDir > 95.0f) {
          joyDir = 95.0f;
          preSelectedGroupID = 0;
        } else if (joyDir < -95.0f) {
          joyDir = -95.0f;
          preSelectedGroupID = 6;
        }
      }
    } else {
      freezeStatus_ = 0;
    }

    if (freezeStatus_ == 1 && autonomyMode_) {
      relativeGoalDis = 0;
      joyDir = 0;
    }

    // --- Path search ---
    float defPathScale = pathScale_;
    if (pathScaleBySpeed_) pathScale_ = defPathScale * joySpeed_;
    if (pathScale_ < minPathScale_) pathScale_ = minPathScale_;

    bool pathFound = false;
    int plannerCloudCropSize = plannerCloudCrop_->points.size();

    while (pathScale_ >= minPathScale_ && pathRange >= minPathRange_) {
      // Reset scores
      for (int i = 0; i < 36 * PATH_NUM; i++) {
        clearPathList_[i] = 0;
        pathPenaltyList_[i] = 0;
      }
      for (int i = 0; i < 36 * GROUP_NUM; i++) {
        clearPathPerGroupScore_[i] = 0;
        clearPathPerGroupNum_[i] = 0;
        pathPenaltyPerGroupScore_[i] = 0;
      }

      float minObsAngCW = -180.0f;
      float minObsAngCCW = 180.0f;
      float diameter = std::sqrt(vehicleLength_ / 2.0f * vehicleLength_ / 2.0f +
                                  vehicleWidth_ / 2.0f * vehicleWidth_ / 2.0f);
      float angOffset = std::atan2(vehicleWidth_, vehicleLength_) * 180.0f / PI;

      const float gridVoxelSize = 0.02f;
      const float searchRadius = 0.45f;
      const float gridVoxelOffsetX = 3.2f;
      const float gridVoxelOffsetY = 4.5f;

      for (int i = 0; i < plannerCloudCropSize; i++) {
        float x = plannerCloudCrop_->points[i].x / pathScale_;
        float y = plannerCloudCrop_->points[i].y / pathScale_;
        float h = plannerCloudCrop_->points[i].intensity;
        float dis = std::sqrt(x * x + y * y);

        if (dis < pathRange / pathScale_ &&
            (dis <= (relativeGoalDis + goalClearRange_) / pathScale_ || !pathCropByGoal_) &&
            checkObstacle_) {
          for (int rotDir = 0; rotDir < 36; rotDir++) {
            float rotAng = (10.0f * rotDir - 180.0f) * PI / 180.0f;
            float angDiff = std::abs(joyDir - (10.0f * rotDir - 180.0f));
            if (angDiff > 180.0f) angDiff = 360.0f - angDiff;

            if ((angDiff > dirThre_ && !dirToVehicle_) ||
                (std::abs(10.0f * rotDir - 180.0f) > dirThre_ && std::abs(joyDir) <= 90.0f && dirToVehicle_) ||
                ((10.0f * rotDir > dirThre_ && 360.0f - 10.0f * rotDir > dirThre_) && std::abs(joyDir) > 90.0f && dirToVehicle_)) {
              continue;
            }

            float x2 = std::cos(rotAng) * x + std::sin(rotAng) * y;
            float y2 = -std::sin(rotAng) * x + std::cos(rotAng) * y;

            float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY *
                           (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

            int indX = static_cast<int>((gridVoxelOffsetX + gridVoxelSize / 2.0f - x2) / gridVoxelSize);
            int indY = static_cast<int>((gridVoxelOffsetY + gridVoxelSize / 2.0f - y2 / scaleY) / gridVoxelSize);

            if (indX >= 0 && indX < GRID_VIXEL_NUM_X && indY >= 0 && indY < GRID_VIXEL_NUM_Y) {
              int ind = GRID_VIXEL_NUM_Y * indX + indY;
              int blockedNum = correspondences_[ind].size();
              for (int j = 0; j < blockedNum; j++) {
                if (h > obstacleHeightThre_ || !useTerrainAnalysis_) {
                  clearPathList_[PATH_NUM * rotDir + correspondences_[ind][j]]++;
                } else {
                  if (pathPenaltyList_[PATH_NUM * rotDir + correspondences_[ind][j]] < h &&
                      h > groundHeightThre_) {
                    pathPenaltyList_[PATH_NUM * rotDir + correspondences_[ind][j]] = h;
                  }
                }
              }
            }
          }
        }

        if (dis < diameter / pathScale_ &&
            (std::abs(x) > vehicleLength_ / pathScale_ / 2.0f ||
             std::abs(y) > vehicleWidth_ / pathScale_ / 2.0f) &&
            (h > obstacleHeightThre_ || !useTerrainAnalysis_) && checkRotObstacle_) {
          float angObs = std::atan2(y, x) * 180.0f / PI;
          if (angObs > 0) {
            if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
            if (minObsAngCW < angObs + angOffset - 180.0f) minObsAngCW = angObs + angOffset - 180.0f;
          } else {
            if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
            if (minObsAngCCW > 180.0f + angObs - angOffset) minObsAngCCW = 180.0f + angObs - angOffset;
          }
        }
      }

      if (minObsAngCW > 0) minObsAngCW = 0;
      if (minObsAngCCW < 0) minObsAngCCW = 0;

      // Score paths
      for (int i = 0; i < 36 * PATH_NUM; i++) {
        int rotDir = i / PATH_NUM;
        float angDiff = std::abs(joyDir - (10.0f * rotDir - 180.0f));
        if (angDiff > 180.0f) angDiff = 360.0f - angDiff;

        if ((angDiff > dirThre_ && !dirToVehicle_) ||
            (std::abs(10.0f * rotDir - 180.0f) > dirThre_ && std::abs(joyDir) <= 90.0f && dirToVehicle_) ||
            ((10.0f * rotDir > dirThre_ && 360.0f - 10.0f * rotDir > dirThre_) && std::abs(joyDir) > 90.0f && dirToVehicle_)) {
          continue;
        }

        if (clearPathList_[i] < pointPerPathThre_) {
          float dirDiff = std::abs(joyDir - endDirPathList_[i % PATH_NUM] - (10.0f * rotDir - 180.0f));
          if (dirDiff > 360.0f) dirDiff -= 360.0f;
          if (dirDiff > 180.0f) dirDiff = 360.0f - dirDiff;

          float rotDirW;
          if (rotDir < 18) rotDirW = std::abs(std::abs(rotDir - 9) + 1);
          else rotDirW = std::abs(std::abs(rotDir - 27) + 1);
          float groupDirW = 4.0f - std::abs(pathList_[i % PATH_NUM] - 3);
          float score = (1.0f - std::sqrt(std::sqrt(dirWeight_ * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW;
          if (relativeGoalDis < omniDirGoalThre_) score = (1.0f - std::sqrt(std::sqrt(dirWeight_ * dirDiff))) * groupDirW * groupDirW;
          if (score > 0) {
            clearPathPerGroupScore_[GROUP_NUM * rotDir + pathList_[i % PATH_NUM]] += score;
            clearPathPerGroupNum_[GROUP_NUM * rotDir + pathList_[i % PATH_NUM]]++;
            pathPenaltyPerGroupScore_[GROUP_NUM * rotDir + pathList_[i % PATH_NUM]] += pathPenaltyList_[i];
          }
        }
      }

      // Select best group
      int selectedGroupID = -1;
      if (preSelectedGroupID >= 0) {
        selectedGroupID = preSelectedGroupID;
      } else {
        float maxScore = 0;
        for (int i = 0; i < 36 * GROUP_NUM; i++) {
          int rotDir = i / GROUP_NUM;
          float rotAng = (10.0f * rotDir - 180.0f) * PI / 180.0f;
          float rotDeg = 10.0f * rotDir;
          if (rotDeg > 180.0f) rotDeg -= 360.0f;

          if (maxScore < clearPathPerGroupScore_[i] &&
              ((rotAng * 180.0f / PI > minObsAngCW && rotAng * 180.0f / PI < minObsAngCCW) ||
               (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive_) ||
               !checkRotObstacle_)) {
            maxScore = clearPathPerGroupScore_[i];
            selectedGroupID = i;
          }
        }
      }

      // Slow-down level
      if (selectedGroupID >= 0) {
        int selectedPathNum = clearPathPerGroupNum_[selectedGroupID];
        float penaltyScore = 0;
        if (selectedPathNum > 0) {
          penaltyScore = pathPenaltyPerGroupScore_[selectedGroupID] / selectedPathNum;
        }

        std_msgs::msg::Int8 slow;
        if (penaltyScore > costHeightThre1_) slow.data = 1;
        else if (penaltyScore > costHeightThre2_) slow.data = 2;
        else if (selectedPathNum < slowPathNumThre_ && std::abs(selectedGroupID - 129) > slowGroupNumThre_) slow.data = 3;
        else slow.data = 0;
        pub_slow_down_->publish(slow);
      }

      // Build output path
      if (selectedGroupID >= 0) {
        int rotDir = selectedGroupID / GROUP_NUM;
        float rotAng = (10.0f * rotDir - 180.0f) * PI / 180.0f;

        int groupID = selectedGroupID % GROUP_NUM;
        int selectedPathLength = startPaths_[groupID]->points.size();

        nav_msgs::msg::Path path;
        path.poses.resize(selectedPathLength);
        int validCount = 0;
        for (int i = 0; i < selectedPathLength; i++) {
          float x = startPaths_[groupID]->points[i].x;
          float y = startPaths_[groupID]->points[i].y;
          float z = startPaths_[groupID]->points[i].z;
          float dis = std::sqrt(x * x + y * y);

          if (dis <= pathRange / pathScale_ && dis <= relativeGoalDis / pathScale_) {
            path.poses[validCount].pose.position.x = pathScale_ * (std::cos(rotAng) * x - std::sin(rotAng) * y);
            path.poses[validCount].pose.position.y = pathScale_ * (std::sin(rotAng) * x + std::cos(rotAng) * y);
            path.poses[validCount].pose.position.z = pathScale_ * z;
            validCount++;
          } else {
            break;
          }
        }
        path.poses.resize(validCount);
        path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime_ * 1e9));
        path.header.frame_id = "base_link";
        pub_path_->publish(path);

        // Free paths visualization
        freePaths_->clear();
        for (int i = 0; i < 36 * PATH_NUM; i++) {
          int rDir = i / PATH_NUM;
          float rAng = (10.0f * rDir - 180.0f) * PI / 180.0f;
          float rDeg = 10.0f * rDir;
          if (rDeg > 180.0f) rDeg -= 360.0f;

          float angDiff = std::abs(joyDir - (10.0f * rDir - 180.0f));
          if (angDiff > 180.0f) angDiff = 360.0f - angDiff;

          if ((angDiff > dirThre_ && !dirToVehicle_) ||
              (std::abs(10.0f * rDir - 180.0f) > dirThre_ && std::abs(joyDir) <= 90.0f && dirToVehicle_) ||
              ((10.0f * rDir > dirThre_ && 360.0f - 10.0f * rDir > dirThre_) && std::abs(joyDir) > 90.0f && dirToVehicle_) ||
              !((rAng * 180.0f / PI > minObsAngCW && rAng * 180.0f / PI < minObsAngCCW) ||
                (rDeg > minObsAngCW && rDeg < minObsAngCCW && twoWayDrive_) || !checkRotObstacle_)) {
            continue;
          }

          if (clearPathList_[i] < pointPerPathThre_) {
            for (const auto & pt : paths_[i % PATH_NUM]->points) {
              float dis = std::sqrt(pt.x * pt.x + pt.y * pt.y);
              if (dis <= pathRange / pathScale_ &&
                  (dis <= (relativeGoalDis + goalClearRange_) / pathScale_ || !pathCropByGoal_)) {
                pcl::PointXYZI point;
                point.x = pathScale_ * (std::cos(rAng) * pt.x - std::sin(rAng) * pt.y);
                point.y = pathScale_ * (std::sin(rAng) * pt.x + std::cos(rAng) * pt.y);
                point.z = pathScale_ * pt.z;
                point.intensity = 1.0f;
                freePaths_->push_back(point);
              }
            }
          }
        }

        sensor_msgs::msg::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths_, freePaths2);
        freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime_ * 1e9));
        freePaths2.header.frame_id = "base_link";
        pub_free_paths_->publish(freePaths2);

        pathFound = true;
        break;
      }

      // Retry with smaller scale/range
      if (pathScale_ >= minPathScale_ + pathScaleStep_) {
        pathScale_ -= pathScaleStep_;
        pathRange = adjacentRange_ * pathScale_ / defPathScale;
      } else {
        pathRange -= pathRangeStep_;
      }
    }

    pathScale_ = defPathScale;

    // No path found — publish zero-length path
    if (!pathFound) {
      nav_msgs::msg::Path path;
      path.poses.resize(1);
      path.poses[0].pose.position.x = 0;
      path.poses[0].pose.position.y = 0;
      path.poses[0].pose.position.z = 0;
      path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime_ * 1e9));
      path.header.frame_id = "base_link";
      pub_path_->publish(path);

      freePaths_->clear();
      sensor_msgs::msg::PointCloud2 freePaths2;
      pcl::toROSMsg(*freePaths_, freePaths2);
      freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime_ * 1e9));
      freePaths2.header.frame_id = "base_link";
      pub_free_paths_->publish(freePaths2);
    }
  }

  // ---- parameters ----
  std::string pathFolder_;
  double vehicleLength_, vehicleWidth_;
  double sensorOffsetX_, sensorOffsetY_;
  double vehicleLengthSlot_, vehicleWidthMargin_, marginYawRateRatio_;
  bool twoWayDrive_;
  double laserVoxelSize_, terrainVoxelSize_;
  bool useTerrainAnalysis_, checkObstacle_, checkRotObstacle_;
  double adjacentRange_;
  double obstacleHeightThre_, groundHeightThre_;
  double costHeightThre1_, costHeightThre2_;
  bool useCost_;
  int slowPathNumThre_, slowGroupNumThre_, surPointThre_;
  int pointPerPathThre_;
  double minRelZ_, maxRelZ_;
  double maxSpeed_;
  double dirWeight_, dirThre_;
  bool dirToVehicle_;
  double pathScale_, minPathScale_, pathScaleStep_;
  bool pathScaleBySpeed_;
  double minPathRange_, pathRangeStep_;
  bool pathRangeBySpeed_, pathCropByGoal_;
  bool autonomyMode_;
  double autonomySpeed_, joyToSpeedDelay_, joyToCheckObstacleDelay_;
  double freezeAng_, freezeTime_;
  double omniDirGoalThre_;
  double goalClearRange_, goalBehindRange_;
  double goalX_, goalY_;

  // New params for TF / input configuration
  std::string global_frame_id_;
  bool use_planned_path_;
  bool use_laser_scan_;
  double waypoint_lookahead_;
  double waypoint_tolerance_;

  // ---- state ----
  float joySpeed_ = 0, joySpeedRaw_ = 0, joyDir_ = 0;
  float vehicleX_ = 0, vehicleY_ = 0, vehicleZ_ = 0;
  float vehicleRoll_ = 0, vehiclePitch_ = 0, vehicleYaw_ = 0, vehicleYawRate_ = 0;
  double odomTime_ = 0, joyTime_ = 0;
  double freezeStartTime_ = 0;
  int freezeStatus_ = 0;
  bool newLaserCloud_ = false, newTerrainCloud_ = false;

  // /planned_path waypoint management
  std::vector<std::tuple<double, double, double>> planned_waypoints_;
  size_t current_wp_idx_ = 0;
  bool navigating_ = false;
  bool has_goal_ = false;

  // ---- path data ----
  pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths_[GROUP_NUM];
  pcl::PointCloud<pcl::PointXYZI>::Ptr paths_[PATH_NUM];
  int pathList_[PATH_NUM] = {};
  float endDirPathList_[PATH_NUM] = {};
  int clearPathList_[36 * PATH_NUM] = {};
  float pathPenaltyList_[36 * PATH_NUM] = {};
  float clearPathPerGroupScore_[36 * GROUP_NUM] = {};
  int clearPathPerGroupNum_[36 * GROUP_NUM] = {};
  float pathPenaltyPerGroupScore_[36 * GROUP_NUM] = {};
  std::vector<int> correspondences_[GRID_VIXEL_NUM];

  // ---- point clouds ----
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths_{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter_, terrainDwzFilter_;

  // ---- pub/sub ----
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_laser_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_terrain_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joystick_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_planned_path_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_nav_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_boundary_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_added_obstacles_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_check_obstacle_;

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_slow_down_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_sur_block_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_free_paths_;

  rclcpp::TimerBase::SharedPtr process_timer_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace local_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<local_planner::LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
