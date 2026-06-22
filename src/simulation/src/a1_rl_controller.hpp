#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <onnxruntime_cxx_api.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace simulation {

class A1RLController : public gazebo::ModelPlugin {
public:
    A1RLController();
    ~A1RLController() override;
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
    void OnUpdate(const gazebo::common::UpdateInfo& info);
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void LoadPolicy(const std::string& path);
    void RefreshObservation();
    void PublishState(const gazebo::common::Time& now);
    bool CheckSafety();

    // Gazebo
    gazebo::physics::ModelPtr model_;
    gazebo::event::ConnectionPtr update_connection_;
    std::vector<gazebo::physics::JointPtr> joints_;
    ignition::math::Pose3d initial_pose_;

    // ROS 2
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string odom_frame_id_ = "odom";
    std::string robot_base_frame_ = "base_footprint";

    // ONNX Runtime
    std::unique_ptr<Ort::Env> ort_env_;
    std::unique_ptr<Ort::Session> ort_session_;
    std::unique_ptr<Ort::AllocatorWithDefaultOptions> ort_allocator_;
    std::string input_name_;
    std::string output_name_;
    std::vector<int64_t> input_shape_ = {1, 225};
    Ort::MemoryInfo memory_info_{nullptr};

    // State machine
    enum State { STAND_UP, RL_RUNNING };
    State state_ = STAND_UP;

    // Timing
    gazebo::common::Time last_infer_time_;
    gazebo::common::Time last_publish_time_;
    double infer_duration_ = 0.02;
    double stand_up_duration_ = 0.5;
    double stand_up_start_time_ = 0.0;
    double publish_duration_ = 0.005;  // 200Hz

    // Joint reindex: Gazebo order (FR,FL,RR,RL) → RL model order (FL,FR,RL,RR)
    static constexpr int reindex_[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};

    // Default joint positions in RL model order (FL, FR, RL, RR)
    std::array<float, 12> default_dof_pos_ = {
        -0.15f, 0.55f, -1.5f,   // FL: hip, thigh, calf
         0.15f, 0.55f, -1.5f,   // FR: hip, thigh, calf
        -0.15f, 0.70f, -1.5f,   // RL: hip, thigh, calf
         0.15f, 0.70f, -1.5f    // RR: hip, thigh, calf
    };

    // Observation history: 5 steps × 45 dims
    static constexpr int HISTORY_LEN = 5;
    static constexpr int OBS_DIM = 45;
    std::array<float, HISTORY_LEN * OBS_DIM> obs_history_{};

    // Last actions for observation
    std::array<float, 12> last_actions_{};

    // Cached PD targets (Gazebo joint order) — applied every step
    std::array<float, 12> target_q_{};

    // Stand-up start positions (Gazebo joint order)
    std::array<float, 12> start_pos_{};

    // Latest cmd_vel
    double cmd_linear_x_ = 0.0;
    double cmd_linear_y_ = 0.0;
    double cmd_angular_z_ = 0.0;

    // cmd_vel timeout: zero velocity if no message received within this duration
    double cmd_vel_timeout_ = 0.5;
    double last_cmd_vel_time_ = 0.0;

    // PD gains
    double kp_stand_up_ = 40.0;
    double kd_stand_up_ = 0.5;
    double kp_rl_ = 80.0;
    double kd_rl_ = 1.0;

    // Safety: max trunk tilt (radians)
    double max_tilt_ = 60.0 * 3.1415926535897931 / 180.0;
};

}  // namespace simulation
