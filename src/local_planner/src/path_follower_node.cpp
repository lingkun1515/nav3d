#include <cmath>
#include <cstring>
#include <chrono>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#ifdef SERIAL_ENABLED
#include "serial/serial.h"
#endif

namespace local_planner
{

constexpr double PI = 3.1415926;

class PathFollowerNode : public rclcpp::Node
{
public:
  PathFollowerNode() : Node("pathFollower")
  {
    declare_parameters();

    // TF buffer (always created; only used when use_global_path)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    setup_pub_sub();

    RCLCPP_INFO(get_logger(), "PathFollower initialization complete.");

    process_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() { process_loop(); });
  }

  ~PathFollowerNode()
  {
#ifdef SERIAL_ENABLED
    if (serial_open_) {
      motor_ctr_serial_.close();
    }
#endif
  }

private:
  void declare_parameters()
  {
    declare_parameter("realRobot", false);
    declare_parameter("serialPort", "/dev/ttyACM0");
    declare_parameter("baudrate", 115200);
    declare_parameter("pubSkipNum", 1);
    declare_parameter("twoWayDrive", true);
    declare_parameter("lookAheadDis", 0.5);
    declare_parameter("yawRateGain", 7.5);
    declare_parameter("stopYawRateGain", 7.5);
    declare_parameter("maxYawRate", 45.0);
    declare_parameter("maxSpeed", 1.0);
    declare_parameter("maxAccel", 1.0);
    declare_parameter("switchTimeThre", 1.0);
    declare_parameter("dirDiffThre", 0.1);
    declare_parameter("omniDirGoalThre", 1.0);
    declare_parameter("omniDirDiffThre", 1.5);
    declare_parameter("stopDisThre", 0.2);
    declare_parameter("slowDwnDisThre", 1.0);
    declare_parameter("useInclRateToSlow", false);
    declare_parameter("inclRateThre", 120.0);
    declare_parameter("slowRate1", 0.25);
    declare_parameter("slowRate2", 0.5);
    declare_parameter("slowRate3", 0.75);
    declare_parameter("slowTime1", 2.0);
    declare_parameter("slowTime2", 2.0);
    declare_parameter("useSideAvoid", false);
    declare_parameter("useInclToStop", false);
    declare_parameter("inclThre", 45.0);
    declare_parameter("stopTime", 5.0);
    declare_parameter("noRotAtStop", false);
    declare_parameter("noRotAtGoal", true);
    declare_parameter("autonomyMode", false);
    declare_parameter("autonomySpeed", 1.0);
    declare_parameter("joyToSpeedDelay", 2.0);
    declare_parameter("cmdVelTimeout", 0.2);
    // Global-path mode (bypass latticePlanner)
    declare_parameter("use_global_path", false);
    declare_parameter("waypoint_lookahead", 2.5);
    declare_parameter("waypoint_tolerance", 0.5);
  }

  void setup_pub_sub()
  {
    auto qos = rclcpp::QoS(5);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odometry_callback(msg); });

    // Always subscribe to both — use_global_path is resolved at runtime
    // (ROS 2 sets parameters after ctor, so we can't branch on it here).
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      "/path", qos,
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) { path_callback(msg); });

    sub_global_path_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", qos,
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) { global_path_callback(msg); });

    sub_joystick_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", qos,
      [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) { joystick_callback(msg); });

    sub_speed_ = create_subscription<std_msgs::msg::Float32>(
      "/speed", qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr msg) { speed_callback(msg); });

    sub_stop_ = create_subscription<std_msgs::msg::Int8>(
      "/stop", qos,
      [this](std_msgs::msg::Int8::ConstSharedPtr msg) { stop_callback(msg); });

    sub_stop_nav_ = create_subscription<std_msgs::msg::Bool>(
      "/stop_navigation", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) { stop_navigation_callback(msg); });

    sub_slow_down_ = create_subscription<std_msgs::msg::Int8>(
      "/slow_down", qos,
      [this](std_msgs::msg::Int8::ConstSharedPtr msg) { slow_down_callback(msg); });

    sub_sur_block_ = create_subscription<std_msgs::msg::Int8>(
      "/surrounding_block", qos,
      [this](std_msgs::msg::Int8::ConstSharedPtr msg) { sur_block_callback(msg); });

    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
  }

  void read_params()
  {
    real_robot_ = get_parameter("realRobot").as_bool();
    serial_port_ = get_parameter("serialPort").as_string();
    baudrate_ = get_parameter("baudrate").as_int();
    pub_skip_num_ = get_parameter("pubSkipNum").as_int();
    two_way_drive_ = get_parameter("twoWayDrive").as_bool();
    look_ahead_dis_ = get_parameter("lookAheadDis").as_double();
    yaw_rate_gain_ = get_parameter("yawRateGain").as_double();
    stop_yaw_rate_gain_ = get_parameter("stopYawRateGain").as_double();
    max_yaw_rate_ = get_parameter("maxYawRate").as_double();
    max_speed_ = get_parameter("maxSpeed").as_double();
    max_accel_ = get_parameter("maxAccel").as_double();
    switch_time_thre_ = get_parameter("switchTimeThre").as_double();
    dir_diff_thre_ = get_parameter("dirDiffThre").as_double();
    omni_dir_goal_thre_ = get_parameter("omniDirGoalThre").as_double();
    omni_dir_diff_thre_ = get_parameter("omniDirDiffThre").as_double();
    stop_dis_thre_ = get_parameter("stopDisThre").as_double();
    slow_dwn_dis_thre_ = get_parameter("slowDwnDisThre").as_double();
    use_incl_rate_to_slow_ = get_parameter("useInclRateToSlow").as_bool();
    incl_rate_thre_ = get_parameter("inclRateThre").as_double();
    slow_rate1_ = get_parameter("slowRate1").as_double();
    slow_rate2_ = get_parameter("slowRate2").as_double();
    slow_rate3_ = get_parameter("slowRate3").as_double();
    slow_time1_ = get_parameter("slowTime1").as_double();
    slow_time2_ = get_parameter("slowTime2").as_double();
    use_side_avoid_ = get_parameter("useSideAvoid").as_bool();
    use_incl_to_stop_ = get_parameter("useInclToStop").as_bool();
    incl_thre_ = get_parameter("inclThre").as_double();
    stop_time_ = get_parameter("stopTime").as_double();
    no_rot_at_stop_ = get_parameter("noRotAtStop").as_bool();
    no_rot_at_goal_ = get_parameter("noRotAtGoal").as_bool();
    autonomy_mode_ = get_parameter("autonomyMode").as_bool();
    autonomy_speed_ = get_parameter("autonomySpeed").as_double();
    joy_to_speed_delay_ = get_parameter("joyToSpeedDelay").as_double();
    cmd_vel_timeout_ = get_parameter("cmdVelTimeout").as_double();

    use_global_path_ = get_parameter("use_global_path").as_bool();
    waypoint_lookahead_ = get_parameter("waypoint_lookahead").as_double();
    waypoint_tolerance_ = get_parameter("waypoint_tolerance").as_double();

    if (autonomy_mode_) {
      joy_speed_ = autonomy_speed_ / max_speed_;
      if (joy_speed_ < 0) joy_speed_ = 0;
      else if (joy_speed_ > 1.0f) joy_speed_ = 1.0f;
    }
  }

  // ---- callbacks ----

  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    odom_time_ = now().seconds();
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicle_roll_ = roll;
    vehicle_pitch_ = pitch;
    vehicle_yaw_ = yaw;
    vehicle_x_ = odom->pose.pose.position.x;
    vehicle_y_ = odom->pose.pose.position.y;
    vehicle_z_ = odom->pose.pose.position.z;
    odom_frame_id_ = odom->header.frame_id;

    if (use_incl_to_stop_) {
      double effective_thre = incl_thre_;
      if (std::abs(roll) > effective_thre * PI / 180.0 ||
          std::abs(pitch) > effective_thre * PI / 180.0) {
        stop_init_time_ = now().seconds();
      }
    }

    if ((std::abs(odom->twist.twist.angular.x) > incl_rate_thre_ * PI / 180.0 ||
         std::abs(odom->twist.twist.angular.y) > incl_rate_thre_ * PI / 180.0) && use_incl_rate_to_slow_) {
      slow_init_time_ = now().seconds();
    }
  }

  void path_callback(const nav_msgs::msg::Path::ConstSharedPtr path_in)
  {
    int path_size = path_in->poses.size();
    path_.poses.resize(path_size);
    for (int i = 0; i < path_size; i++) {
      path_.poses[i].pose.position.x = path_in->poses[i].pose.position.x;
      path_.poses[i].pose.position.y = path_in->poses[i].pose.position.y;
      path_.poses[i].pose.position.z = path_in->poses[i].pose.position.z;
    }

    vehicle_x_rec_ = vehicle_x_;
    vehicle_y_rec_ = vehicle_y_;
    vehicle_z_rec_ = vehicle_z_;
    vehicle_roll_rec_ = vehicle_roll_;
    vehicle_pitch_rec_ = vehicle_pitch_;
    vehicle_yaw_rec_ = vehicle_yaw_;

    path_point_id_ = 0;
    path_init_ = true;
    last_path_time_ = now().seconds();
  }

  void global_path_callback(const nav_msgs::msg::Path::ConstSharedPtr path_in)
  {
    if (path_in->poses.empty()) {
      global_waypoints_.clear();
      current_wp_idx_ = 0;
      navigating_ = false;
      path_init_ = false;
      RCLCPP_INFO(get_logger(), "Global path cleared.");
      return;
    }

    global_waypoints_.clear();
    for (const auto & pose : path_in->poses) {
      global_waypoints_.push_back({{
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z
      }});
    }

    current_wp_idx_ = 0;
    navigating_ = true;
    path_init_ = true;
    last_path_time_ = now().seconds();
    RCLCPP_INFO(get_logger(), "Global path received: %zu waypoints.",
                global_waypoints_.size());
  }

  void joystick_callback(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
  {
    joy_time_ = now().seconds();
    joy_speed_raw_ = std::sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
    joy_speed_ = joy_speed_raw_;
    if (joy_speed_ > 1.0f) joy_speed_ = 1.0f;
    if (joy->axes[4] == 0) joy_speed_ = 0;
    joy_yaw_ = joy->axes[3];
    if (joy_speed_ == 0 && no_rot_at_stop_) joy_yaw_ = 0;

    if (joy->axes[4] < 0 && !two_way_drive_) {
      joy_speed_ = 0;
      joy_yaw_ = 0;
    }

    joy_manual_fwd_ = joy->axes[4];
    joy_manual_left_ = joy->axes[3];
    joy_manual_yaw_ = joy->axes[0];

    autonomy_mode_ = (joy->axes[2] <= -0.1);
    manual_mode_ = (joy->axes[5] <= -0.1);
  }

  void speed_callback(const std_msgs::msg::Float32::ConstSharedPtr speed)
  {
    double speed_time = now().seconds();
    if (autonomy_mode_ && speed_time - joy_time_ > joy_to_speed_delay_ && joy_speed_raw_ == 0) {
      joy_speed_ = speed->data / max_speed_;
      if (joy_speed_ < 0) joy_speed_ = 0;
      else if (joy_speed_ > 1.0f) joy_speed_ = 1.0f;
    }
  }

  void stop_callback(const std_msgs::msg::Int8::ConstSharedPtr stop)
  {
    safety_stop_ = stop->data;
  }

  void stop_navigation_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    safety_stop_ = msg->data ? 1 : 0;
  }

  void slow_down_callback(const std_msgs::msg::Int8::ConstSharedPtr slow)
  {
    slow_down_ = slow->data;
  }

  void sur_block_callback(const std_msgs::msg::Int8::ConstSharedPtr block)
  {
    br_block_ = (block->data % 2 >= 1);
    bl_block_ = (block->data % 4 >= 2);
    fr_block_ = (block->data % 8 >= 4);
    fl_block_ = (block->data % 16 >= 8);
    r_block_  = (block->data % 32 >= 16);
    l_block_  = (block->data >= 32);
  }

  // ---- global-path helpers ----

  /// Select the lookahead waypoint, advance reached ones, detect goal.
  /// Returns true if we have a valid target; sets target_x/target_y in map frame.
  bool select_global_waypoint(double & tgt_x, double & tgt_y,
                              int & path_size, double & end_dist,
                              double & target_dist)
  {
    if (!navigating_ || global_waypoints_.empty()) return false;

    const size_t N = global_waypoints_.size();

    // 1. Advance reached waypoints
    while (current_wp_idx_ < N - 1) {
      double wx = global_waypoints_[current_wp_idx_][0];
      double wy = global_waypoints_[current_wp_idx_][1];
      double d = std::hypot(wx - vehicle_x_, wy - vehicle_y_);
      if (d < waypoint_tolerance_) {
        current_wp_idx_++;
      } else {
        break;
      }
    }

    // 2. Lookahead: find first waypoint beyond lookahead distance
    size_t target_idx = current_wp_idx_;
    for (size_t i = current_wp_idx_; i < N; i++) {
      double wx = global_waypoints_[i][0];
      double wy = global_waypoints_[i][1];
      double d = std::hypot(wx - vehicle_x_, wy - vehicle_y_);
      if (d > waypoint_lookahead_) {
        target_idx = i;
        break;
      }
      target_idx = i;
    }

    // 3. Rear guard: if target is behind vehicle in vehicle frame,
    //    back-track to the farthest-forward waypoint still ahead.
    {
      double dx = global_waypoints_[target_idx][0] - vehicle_x_;
      double dy = global_waypoints_[target_idx][1] - vehicle_y_;
      double rel_x =  std::cos(vehicle_yaw_) * dx + std::sin(vehicle_yaw_) * dy;
      // double rel_y = -std::sin(vehicle_yaw_) * dx + std::cos(vehicle_yaw_) * dy;

      if (rel_x < 0.0 && target_idx > current_wp_idx_) {
        for (int i = static_cast<int>(target_idx) - 1; i >= static_cast<int>(current_wp_idx_); i--) {
          double dx2 = global_waypoints_[i][0] - vehicle_x_;
          double dy2 = global_waypoints_[i][1] - vehicle_y_;
          double rel_x2 = std::cos(vehicle_yaw_) * dx2 + std::sin(vehicle_yaw_) * dy2;
          if (rel_x2 >= 0.0) {
            target_idx = static_cast<size_t>(i);
            break;
          }
        }
      }
    }

    // 4. Goal reached detection
    if (current_wp_idx_ >= N - 1) {
      double wx = global_waypoints_.back()[0];
      double wy = global_waypoints_.back()[1];
      double d = std::hypot(wx - vehicle_x_, wy - vehicle_y_);
      if (d < waypoint_tolerance_) {
        navigating_ = false;
        RCLCPP_INFO(get_logger(), "Global path goal reached.");
        return false;
      }
    }

    tgt_x = global_waypoints_[target_idx][0];
    tgt_y = global_waypoints_[target_idx][1];

    // End distance (to last waypoint)
    double ex = global_waypoints_.back()[0] - vehicle_x_;
    double ey = global_waypoints_.back()[1] - vehicle_y_;
    end_dist = std::hypot(ex, ey);

    // Distance to target
    double tx = tgt_x - vehicle_x_;
    double ty = tgt_y - vehicle_y_;
    target_dist = std::hypot(tx, ty);

    // Path "size" for control logic: > 1 means we have a target
    path_size = static_cast<int>(N);

    return true;
  }

  // ---- main control loop (100 Hz) ----

  void process_loop()
  {
    read_params();

    if (!path_init_) return;

    float disX, disY, dis;
    float endDisX, endDisY, endDis;
    int pathSize;
    float dirDiff;

    if (use_global_path_) {
      // ---- Global-path mode: select waypoint + transform to vehicle frame ----
      double tgt_x, tgt_y, end_dist, tgt_dist;
      int wp_count;
      if (!select_global_waypoint(tgt_x, tgt_y, wp_count, end_dist, tgt_dist)) {
        // No valid target → stop
        vehicle_speed_ = 0.0;
        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        pub_cmd_vel_->publish(cmd_vel);
        return;
      }

      // Transform waypoint from map → base_link
      double local_x = 0.0, local_y = 0.0;
      try {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
          "base_link", "map", tf2::TimePointZero, tf2::durationFromSec(0.1));

        geometry_msgs::msg::PointStamped p_in, p_out;
        p_in.header.frame_id = "map";
        p_in.header.stamp = tf.header.stamp;
        p_in.point.x = tgt_x;
        p_in.point.y = tgt_y;
        p_in.point.z = 0.0;
        tf2::doTransform(p_in, p_out, tf);
        local_x = p_out.point.x;
        local_y = p_out.point.y;
      } catch (const tf2::TransformException & e) {
        // Fall back: manual odometry-based transform
        double dx = tgt_x - vehicle_x_;
        double dy = tgt_y - vehicle_y_;
        local_x =  std::cos(vehicle_yaw_) * dx + std::sin(vehicle_yaw_) * dy;
        local_y = -std::sin(vehicle_yaw_) * dx + std::cos(vehicle_yaw_) * dy;
      }

      // Compute pure-pursuit params in vehicle frame
      disX = static_cast<float>(local_x);
      disY = static_cast<float>(local_y);
      dis = std::sqrt(disX * disX + disY * disY);
      endDis = static_cast<float>(end_dist);
      pathSize = (dis > 0.01) ? 2 : 1;  // 2 = have valid target; 1 = at target → stop

      // dirDiff sign convention: the common control law does -gain*dirDiff.
      // atan2(disY,disX) gives the bearing to target in vehicle frame
      // (positive=left).  To turn TOWARD the target we need +gain*bearing,
      // so negate here so -gain*(-bearing) = +gain*bearing.
      dirDiff = -std::atan2(disY, disX);
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

    } else {
      // ---- Local path mode (existing logic) ----
      float vehicleXRel = std::cos(vehicle_yaw_rec_) * (vehicle_x_ - vehicle_x_rec_)
                        + std::sin(vehicle_yaw_rec_) * (vehicle_y_ - vehicle_y_rec_);
      float vehicleYRel = -std::sin(vehicle_yaw_rec_) * (vehicle_x_ - vehicle_x_rec_)
                        + std::cos(vehicle_yaw_rec_) * (vehicle_y_ - vehicle_y_rec_);

      pathSize = static_cast<int>(path_.poses.size());
      endDisX = path_.poses[pathSize - 1].pose.position.x - vehicleXRel;
      endDisY = path_.poses[pathSize - 1].pose.position.y - vehicleYRel;
      endDis = std::sqrt(endDisX * endDisX + endDisY * endDisY);

      while (path_point_id_ < pathSize - 1) {
        disX = path_.poses[path_point_id_].pose.position.x - vehicleXRel;
        disY = path_.poses[path_point_id_].pose.position.y - vehicleYRel;
        dis = std::sqrt(disX * disX + disY * disY);
        if (dis < look_ahead_dis_) {
          path_point_id_++;
        } else {
          break;
        }
      }

      disX = path_.poses[path_point_id_].pose.position.x - vehicleXRel;
      disY = path_.poses[path_point_id_].pose.position.y - vehicleYRel;
      dis = std::sqrt(disX * disX + disY * disY);
      float pathDir = std::atan2(disY, disX);

      dirDiff = vehicle_yaw_ - vehicle_yaw_rec_ - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
    }

    // ---- Common control logic (both modes) ----

    if (two_way_drive_) {
      double time = now().seconds();
      if (std::abs(dirDiff) > PI / 2 && nav_fwd_ && time - switch_time_ > switch_time_thre_) {
        nav_fwd_ = false;
        switch_time_ = time;
      } else if (std::abs(dirDiff) < PI / 2 && !nav_fwd_ && time - switch_time_ > switch_time_thre_) {
        nav_fwd_ = true;
        switch_time_ = time;
      }
    }

    float joySpeed2 = max_speed_ * joy_speed_;
    if (!nav_fwd_) {
      dirDiff += PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      joySpeed2 *= -1;
    }

    float vehicleYawRate;
    if (std::abs(vehicle_speed_) < 2.0 * max_accel_ / 100.0)
      vehicleYawRate = -stop_yaw_rate_gain_ * dirDiff;
    else
      vehicleYawRate = -yaw_rate_gain_ * dirDiff;

    if (vehicleYawRate > max_yaw_rate_ * PI / 180.0) vehicleYawRate = max_yaw_rate_ * PI / 180.0;
    else if (vehicleYawRate < -max_yaw_rate_ * PI / 180.0) vehicleYawRate = -max_yaw_rate_ * PI / 180.0;

    if (joySpeed2 == 0 && !autonomy_mode_) {
      vehicleYawRate = max_yaw_rate_ * joy_yaw_ * PI / 180.0;
    } else if (pathSize <= 1 || (dis < stop_dis_thre_ && no_rot_at_goal_)) {
      vehicleYawRate = 0;
    }

    if (pathSize <= 1) {
      joySpeed2 = 0;
    } else if (endDis / slow_dwn_dis_thre_ < joy_speed_) {
      joySpeed2 *= endDis / slow_dwn_dis_thre_;
    }

    float joySpeed3 = joySpeed2;
    if (!use_global_path_) {
      if ((odom_time_ < slow_init_time_ + slow_time1_ && slow_init_time_ > 0) || slow_down_ == 1)
        joySpeed3 *= slow_rate1_;
      else if ((odom_time_ < slow_init_time_ + slow_time1_ + slow_time2_ && slow_init_time_ > 0) || slow_down_ == 2)
        joySpeed3 *= slow_rate2_;
      else if (slow_down_ == 3)
        joySpeed3 *= slow_rate3_;
    }

    if ((std::abs(dirDiff) < dir_diff_thre_ ||
         (endDis < omni_dir_goal_thre_ && std::abs(dirDiff) < omni_dir_diff_thre_)) && dis > stop_dis_thre_) {
      if (vehicle_speed_ < joySpeed3) vehicle_speed_ += max_accel_ / 100.0;
      else if (vehicle_speed_ > joySpeed3) vehicle_speed_ -= max_accel_ / 100.0;
    } else {
      if (vehicle_speed_ > 0) vehicle_speed_ -= max_accel_ / 100.0;
      else if (vehicle_speed_ < 0) vehicle_speed_ += max_accel_ / 100.0;
    }

    if (odom_time_ < stop_init_time_ + stop_time_ && stop_init_time_ > 0) {
      vehicle_speed_ = 0;
      vehicleYawRate = 0;
    }

    if (safety_stop_ >= 1) vehicle_speed_ = 0;
    if (safety_stop_ >= 2) vehicleYawRate = 0;

    if (!manual_mode_ && path_init_) {
      double now_sec = now().seconds();
      bool odom_stale = now_sec - odom_time_ > cmd_vel_timeout_;
      bool path_stale = now_sec - last_path_time_ > cmd_vel_timeout_;
      // Global path replans at ~0.67 Hz (replan_period_s=1.5); only
      // apply the path-age timeout for the fast local /path channel.
      if (odom_stale || (!use_global_path_ && path_stale)) {
        vehicle_speed_ = 0;
        vehicleYawRate = 0;
      }
    }

    pub_skip_count_--;
    if (pub_skip_count_ < 0) {
      auto cmd_vel = geometry_msgs::msg::Twist();
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;
      cmd_vel.angular.z = vehicleYawRate;

      if (std::abs(vehicle_speed_) > max_accel_ / 100.0) {
        if (omni_dir_goal_thre_ > 0) {
          cmd_vel.linear.x = std::cos(dirDiff) * vehicle_speed_;
          cmd_vel.linear.y = -std::sin(dirDiff) * vehicle_speed_;
        } else {
          cmd_vel.linear.x = vehicle_speed_;
        }
      } else {
        if (omni_dir_goal_thre_ > 0 && use_side_avoid_ && !use_global_path_) {
          if (vehicleYawRate < 0 && (bl_block_ || fr_block_)) {
            cmd_vel.angular.z = 0;
            if (bl_block_ && !r_block_) {
              cmd_vel.linear.y = -max_speed_ / 2.0;
            } else if (fr_block_ && !l_block_) {
              cmd_vel.linear.y = max_speed_ / 2.0;
            }
          } else if (vehicleYawRate > 0 && (br_block_ || fl_block_)) {
            cmd_vel.angular.z = 0;
            if (fl_block_ && !r_block_) {
              cmd_vel.linear.y = -max_speed_ / 2.0;
            } else if (br_block_ && !l_block_) {
              cmd_vel.linear.y = max_speed_ / 2.0;
            }
          }
        }
      }

      if (manual_mode_) {
        cmd_vel.linear.x = max_speed_ * joy_manual_fwd_;
        if (omni_dir_goal_thre_ > 0) cmd_vel.linear.y = max_speed_ / 2.0 * joy_manual_left_;
        cmd_vel.angular.z = max_yaw_rate_ * PI / 180.0 * joy_manual_yaw_;
      }

      // ---- diagnostic: throttled to ~2 Hz ----
      {
        static int dbg_skip = 0;
        if (++dbg_skip >= 50) {
          dbg_skip = 0;
          RCLCPP_INFO(get_logger(),
            "global=%d dDiff=%.3f dis=%.3f endDis=%.3f pSz=%d "
            "vYaw=%.3f vSpd=%.3f jSpd2=%.3f stop=%d nav=%d "
            "odom_age=%.3f path_age=%.3f "
            "gain_stop=%.2f gain_move=%.2f maxYaw=%.2f maxAcc=%.2f",
            use_global_path_, dirDiff, dis, endDis, pathSize,
            vehicleYawRate, vehicle_speed_, joySpeed2,
            safety_stop_, navigating_,
            now().seconds() - odom_time_,
            now().seconds() - last_path_time_,
            stop_yaw_rate_gain_, yaw_rate_gain_, max_yaw_rate_, max_accel_);
        }
      }
      pub_cmd_vel_->publish(cmd_vel);
      pub_skip_count_ = pub_skip_num_;

#ifdef SERIAL_ENABLED
      if (real_robot_) {
        write_serial(cmd_vel);
      }
#endif
    }
  }

#ifdef SERIAL_ENABLED
  void write_serial(const geometry_msgs::msg::Twist & cmd_vel)
  {
    if (serial_open_) {
      float buffer[3];
      buffer[0] = static_cast<float>(cmd_vel.linear.x);
      buffer[1] = static_cast<float>(cmd_vel.linear.y);
      buffer[2] = static_cast<float>(cmd_vel.angular.z);

      size_t size = sizeof(float);
      uint8_t serial_buffer[3 * sizeof(float) + 1];
      std::memcpy(serial_buffer, buffer, 3 * size);
      serial_buffer[3 * size] = '\n';

      motor_ctr_serial_.write(serial_buffer, 3 * size + 1);
    } else {
      init_frame_count_++;
      if (init_frame_count_ >= 100 && init_frame_count_ % 50 == 0) {
        try {
          motor_ctr_serial_.open();
        } catch (serial::IOException &) {
        }

        if (motor_ctr_serial_.isOpen()) {
          serial_open_ = true;
          RCLCPP_INFO(get_logger(), "Serial port open.");
        } else {
          RCLCPP_INFO(get_logger(), "Opening serial port %s...", serial_port_.c_str());
        }
      }
    }
  }
#endif

  // ---- parameters ----
  bool real_robot_{false};
  std::string serial_port_{"/dev/ttyACM0"};
  int baudrate_{115200};
  int pub_skip_num_{1};
  bool two_way_drive_{true};
  double look_ahead_dis_{0.5};
  double yaw_rate_gain_{7.5}, stop_yaw_rate_gain_{7.5};
  double max_yaw_rate_{45.0};
  double max_speed_{1.0}, max_accel_{1.0};
  double switch_time_thre_{1.0};
  double dir_diff_thre_{0.1};
  double omni_dir_goal_thre_{1.0};
  double omni_dir_diff_thre_{1.5};
  double stop_dis_thre_{0.2};
  double slow_dwn_dis_thre_{1.0};
  bool use_incl_rate_to_slow_{false};
  double incl_rate_thre_{120.0};
  double slow_rate1_{0.25}, slow_rate2_{0.5}, slow_rate3_{0.75};
  double slow_time1_{2.0}, slow_time2_{2.0};
  bool use_side_avoid_{false};
  bool use_incl_to_stop_{false};
  double incl_thre_{45.0};
  double stop_time_{5.0};
  bool no_rot_at_stop_{false}, no_rot_at_goal_{true};
  bool autonomy_mode_{false};
  double autonomy_speed_{1.0};
  double joy_to_speed_delay_{2.0};
  double cmd_vel_timeout_{0.2};

  // Global-path mode
  bool use_global_path_{false};
  double waypoint_lookahead_{2.5};
  double waypoint_tolerance_{0.5};

  // ---- state ----
  float joy_speed_{0}, joy_speed_raw_{0}, joy_yaw_{0};
  double last_path_time_{0};
  float joy_manual_fwd_{0}, joy_manual_left_{0}, joy_manual_yaw_{0};
  bool manual_mode_{false};
  int safety_stop_{0};
  int slow_down_{0};
  bool br_block_{false}, bl_block_{false}, fr_block_{false};
  bool fl_block_{false}, r_block_{false}, l_block_{false};

  float vehicle_x_{0}, vehicle_y_{0}, vehicle_z_{0};
  float vehicle_roll_{0}, vehicle_pitch_{0}, vehicle_yaw_{0};
  float vehicle_x_rec_{0}, vehicle_y_rec_{0}, vehicle_z_rec_{0};
  float vehicle_roll_rec_{0}, vehicle_pitch_rec_{0}, vehicle_yaw_rec_{0};
  float vehicle_speed_{0};

  double odom_time_{0}, joy_time_{0};
  double slow_init_time_{0}, stop_init_time_{0};
  int path_point_id_{0};
  bool path_init_{false};
  bool nav_fwd_{true};
  double switch_time_{0};

  int pub_skip_count_{0};

  nav_msgs::msg::Path path_;

  // Global-path state
  std::vector<std::array<double, 3>> global_waypoints_;
  size_t current_wp_idx_{0};
  bool navigating_{false};
  std::string odom_frame_id_;

#ifdef SERIAL_ENABLED
  serial::Serial motor_ctr_serial_;
  bool serial_open_{false};
  int init_frame_count_{0};
#endif

  // ---- pub/sub ----
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joystick_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_stop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_nav_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_slow_down_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_sur_block_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  rclcpp::TimerBase::SharedPtr process_timer_;

  // TF (used only in global-path mode)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace local_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<local_planner::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
