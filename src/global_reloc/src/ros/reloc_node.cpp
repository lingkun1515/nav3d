// ROS2 relocalization node (Foxy-compatible).
// Subscribes: <lidar_topic> (sensor_msgs/PointCloud2), optional <imu_topic>.
// Publishes: <init_pose_topic> (geometry_msgs/PoseWithCovarianceStamped).
// Triggers reloc when accumulate_frames reached; auto-retriggers on tracking loss.
#include "global_reloc/relocalizer.hpp"
#include "global_reloc/map_builder.hpp"
#include "global_reloc/params.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <filesystem>

using namespace global_reloc;

class RelocNode : public rclcpp::Node {
 public:
  RelocNode() : Node("global_reloc_node") {
    // Parameters.
    this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    this->declare_parameter<std::string>("imu_topic", "/livox/imu");
    this->declare_parameter<std::string>("tracking_state_topic", "/super_lio/tracking_state");
    this->declare_parameter<std::string>("init_pose_topic", "/initial_pose");
    this->declare_parameter<std::string>("map_key_path", "");
    this->declare_parameter<std::string>("params_path", "");
    this->declare_parameter<bool>("auto_retrigger", true);
    this->declare_parameter<double>("min_confidence", 0.30);
    this->declare_parameter<std::string>("confidence_topic", "/reloc_confidence");
    this->declare_parameter<std::string>("reliable_topic", "/reloc_reliable");

    std::string map_key = this->get_parameter("map_key_path").as_string();
    std::string params_path = this->get_parameter("params_path").as_string();
    auto_retrigger_ = this->get_parameter("auto_retrigger").as_bool();

    RelocParams params;
    if (!params_path.empty()) {
      try { params = loadParamsFromFile(params_path); }
      catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "params load failed: %s", e.what());
      }
    }
    // Coarse strategy: params file is the default; ROS param can override.
    this->declare_parameter<std::string>("coarse_strategy", "");
    std::string coarse_strategy = this->get_parameter("coarse_strategy").as_string();
    if (!coarse_strategy.empty()) {
      params.coarse_strategy = coarse_strategy;
    }
    // Node params from the shared params file (not ROS declare_parameter).
    gravity_pitch_deg_ = params.gravity_pitch_deg;
    gate_publish_ = params.gate_publish;

    RCLCPP_INFO(this->get_logger(),
        "params: coarse=%s gate=%d grav_pitch=%.1f accum=%d accum_dt=%.1f ndt_grid=%.1f ndt_yaw=%d",
        params.coarse_strategy.c_str(), gate_publish_, gravity_pitch_deg_,
        params.accumulate_frames, params.accumulate_max_dt,
        params.bev.ndt_grid_step, params.bev.ndt_yaw_count);
    if (map_key.empty()) {
      RCLCPP_ERROR(this->get_logger(), "map_key_path not set; exiting");
      throw std::runtime_error("map_key_path required");
    }
    auto map = std::make_shared<GlobalMap>(loadGlobalMap(map_key));
    RCLCPP_INFO(this->get_logger(), "map loaded: %zu pts", map->size());

    reloc_ = std::make_unique<Relocalizer>(params, map);
    if (gravity_pitch_deg_ != 0.0) {
      double pr = gravity_pitch_deg_ * M_PI / 180.0;
      Eigen::Vector3d g(-std::sin(pr), 0.0, -std::cos(pr));
      reloc_->setGravityAttitude(g);
      RCLCPP_INFO(this->get_logger(), "gravity prior: pitch=%.1f deg  g=(%.3f, %.3f, %.3f)",
                  gravity_pitch_deg_, g.x(), g.y(), g.z());
    }

    std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
    std::string init_pose_topic = this->get_parameter("init_pose_topic").as_string();
    std::string tracking_topic = this->get_parameter("tracking_state_topic").as_string();
    min_confidence_ = this->get_parameter("min_confidence").as_double();
    std::string conf_topic = this->get_parameter("confidence_topic").as_string();
    std::string reliable_topic = this->get_parameter("reliable_topic").as_string();

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, rclcpp::SensorDataQoS(),
        std::bind(&RelocNode::onCloud, this, std::placeholders::_1));
    init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        init_pose_topic, 10);
    conf_pub_ = this->create_publisher<std_msgs::msg::Float64>(conf_topic, 10);
    reliable_pub_ = this->create_publisher<std_msgs::msg::Bool>(reliable_topic, 10);
    track_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        tracking_topic, 10,
        std::bind(&RelocNode::onTrackingState, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "global_reloc ready (sub=%s pub=%s, min_conf=%.2f)",
                lidar_topic.c_str(), init_pose_topic.c_str(), min_confidence_);
  }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(cloud, cloud, idx);
    if (cloud.empty()) return;

    double stamp = rclcpp::Time(msg->header.stamp).seconds();
    bool ready = reloc_->addFrame(cloud, stamp);
    if (!ready) return;

    runReloc();
  }

  void onTrackingState(const std_msgs::msg::Bool::ConstSharedPtr msg) {
    // false => tracking lost => allow retrigger on next accumulated frame.
    tracking_ok_ = msg->data;
    if (auto_retrigger_ && !msg->data) {
      rclcpp::Time now = this->now();
      (void)now;  // cooldown check deferred to actual frame trigger
    }
  }

  void runReloc() {
    RelocResult r = reloc_->estimate();
    static bool strategy_logged = false;
    if (!strategy_logged) {
      RCLCPP_INFO(this->get_logger(), "active strategy: %s  (cands=%d  converged=%d)",
                  reloc_->params().coarse_strategy.c_str(), r.candidates_evaluated, r.converged);
      strategy_logged = true;
    }
    rclcpp::Time now = this->now();
    last_reloc_ = now;
    if (!r.converged) {
      RCLCPP_WARN(this->get_logger(), "reloc failed: %s", r.failure_reason.c_str());
      return;
    }
    Eigen::Vector3d t = r.pose.translation();

    // Reliability via TEMPORAL CONSISTENCY (ground-truth-free, the only signal
    // that survives a repetitive map where per-shot scores are flat). The robot
    // cannot teleport, so a reloc that agrees with the immediately previous
    // reloc (within max_speed*dt) is reliable; an isolated jumping reloc is an
    // outlier. The first reloc is published but flagged unreliable until a
    // second one confirms it. (Validated offline by scripts/verify_reliability.py.)
    bool reliable = false;
    if (have_pending_) {
      double dt = std::max(1e-3, (now - pending_time_).seconds());
      double d = std::hypot(t.x() - pending_xy_.x(), t.y() - pending_xy_.y());
      reliable = d <= max_speed_ * dt + reloc_margin_;
    }
    // The current reloc becomes the predecessor for the next attempt.
    pending_xy_ = t.head<2>();
    pending_time_ = now;
    have_pending_ = true;

    std_msgs::msg::Float64 conf_msg; conf_msg.data = r.confidence; conf_pub_->publish(conf_msg);
    std_msgs::msg::Bool rel_msg; rel_msg.data = reliable; reliable_pub_->publish(rel_msg);

    if (gate_publish_ && !reliable) {
      RCLCPP_WARN(this->get_logger(), "reloc not yet consistent (conf=%.3f) withheld t=[%.2f %.2f %.2f]",
                  r.confidence, t.x(), t.y(), t.z());
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = now;
    out.header.frame_id = "map";
    Eigen::Quaterniond q(r.pose.rotation());
    out.pose.pose.position.x = t.x();
    out.pose.pose.position.y = t.y();
    out.pose.pose.position.z = t.z();
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();
    out.pose.pose.orientation.w = q.w();
    // Covariance: large until the reloc is temporally confirmed.
    double cov = reliable ? 0.05 : 1.0;
    for (int i = 0; i < 6; ++i) out.pose.covariance[i * 6 + i] = cov;
    init_pose_pub_->publish(out);
    RCLCPP_INFO(this->get_logger(),
                "[/initial_pose PUBLISHED] reliable=%d conf=%.3f score=%.3f cov=%.2f t=[%.2f %.2f %.2f]",
                reliable ? 1 : 0, r.confidence, r.score, cov, t.x(), t.y(), t.z());
  }

  std::unique_ptr<Relocalizer> reloc_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr track_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr conf_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reliable_pub_;
  bool tracking_ok_ = true;
  bool auto_retrigger_ = true;
  bool gate_publish_ = false;
  double min_confidence_ = 0.0;   // diagnostic only (per-shot margin; flat w/o intensity)
  double gravity_pitch_deg_ = 0.0;
  double max_speed_ = 2.0;        // [m/s] temporal-consistency motion limit
  double reloc_margin_ = 1.0;     // [m] extra tolerance for consistency check
  bool have_pending_ = false;
  Eigen::Vector2d pending_xy_{0, 0};
  rclcpp::Time pending_time_ = rclcpp::Time(0, 0);
  rclcpp::Time last_reloc_ = rclcpp::Time(0, 0);
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelocNode>());
  rclcpp::shutdown();
  return 0;
}
