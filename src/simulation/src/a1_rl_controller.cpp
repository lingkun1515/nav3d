#include "a1_rl_controller.hpp"

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <algorithm>
#include <cstring>

namespace simulation {

A1RLController::A1RLController() = default;

A1RLController::~A1RLController() = default;

void A1RLController::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    model_ = model;

    // Read joint names from SDF
    if (sdf->HasElement("jointName")) {
        sdf::ElementPtr joint_elem = sdf->GetElement("jointName");
        while (joint_elem) {
            std::string name = joint_elem->Get<std::string>();
            auto joint = model->GetJoint(name);
            if (joint) {
                joints_.push_back(joint);
                RCLCPP_INFO_STREAM(rclcpp::get_logger("a1_rl_controller"),
                                   "Found joint: " << name);
            } else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("a1_rl_controller"),
                                    "Joint not found: " << name);
            }
            joint_elem = joint_elem->GetNextElement("jointName");
        }
    }

    if (joints_.size() != 12) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("a1_rl_controller"),
                            "Expected 12 joints, got " << joints_.size());
        return;
    }

    // Read SDF parameters
    std::string model_path;
    if (sdf->HasElement("modelPath")) {
        model_path = sdf->GetElement("modelPath")->Get<std::string>();
    }
    if (sdf->HasElement("odomFrameId")) {
        odom_frame_id_ = sdf->GetElement("odomFrameId")->Get<std::string>();
    }
    if (sdf->HasElement("robotBaseFrame")) {
        robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    // Resolve model path (package:// → absolute file path)
    if (model_path.find("package://") == 0) {
        std::string relative = model_path.substr(10);  // "pkg/rest/of/path"
        auto pos = relative.find('/');
        if (pos != std::string::npos) {
            std::string package = relative.substr(0, pos);
            std::string rest = relative.substr(pos + 1);
            bool found = false;

            // Search AMENT_PREFIX_PATH for the package's share directory
            const char* ament_prefix = std::getenv("AMENT_PREFIX_PATH");
            if (ament_prefix && ament_prefix[0] != '\0') {
                std::string paths(ament_prefix);
                size_t start = 0, end;
                while ((end = paths.find(':', start)) != std::string::npos) {
                    std::string candidate =
                        paths.substr(start, end - start) + "/share/" + package + "/" + rest;
                    if (std::ifstream(candidate).good()) {
                        model_path = candidate;
                        found = true;
                        break;
                    }
                    start = end + 1;
                }
                if (!found) {
                    std::string candidate =
                        paths.substr(start) + "/share/" + package + "/" + rest;
                    if (std::ifstream(candidate).good()) {
                        model_path = candidate;
                        found = true;
                    }
                }
            }

            // Fallback: try relative to install prefix via GAZEBO_PLUGIN_PATH
            if (!found) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("a1_rl_controller"),
                                   "AMENT_PREFIX_PATH search failed for " << model_path
                                   << ", trying fallback...");
            }
        }
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("a1_rl_controller"),
                       "Loading RL model from: " << model_path);
    try {
        LoadPolicy(model_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("a1_rl_controller"),
                            "Failed to load ONNX model: " << e.what());
        return;
    }

    // Set joints to default standing pose before physics starts.
    // Spawning with joints at 0 (straight legs) causes feet to penetrate
    // the ground, bouncing the robot and triggering safety tilt.
    for (int i = 0; i < 12; i++) {
        int rl_idx = reindex_[i];
        joints_[i]->SetPosition(0, default_dof_pos_[rl_idx]);
        start_pos_[i] = default_dof_pos_[rl_idx];
        target_q_[i] = default_dof_pos_[rl_idx];
    }

    // Record initial pose for odometry
    initial_pose_ = model_->WorldPose();

    // Create ROS 2 node via gazebo_ros
    ros_node_ = gazebo_ros::Node::Get(sdf);

    std::string cmd_vel_topic = "/cmd_vel";
    if (sdf->HasElement("cmdVelTopic")) {
        cmd_vel_topic = sdf->GetElement("cmdVelTopic")->Get<std::string>();
    }
    cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, 10,
        std::bind(&A1RLController::CmdVelCallback, this, std::placeholders::_1));

    if (sdf->HasElement("cmdVelTimeout")) {
        cmd_vel_timeout_ = sdf->GetElement("cmdVelTimeout")->Get<double>();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("a1_rl_controller"),
                           "cmd_vel timeout set to " << cmd_vel_timeout_ << " s");
    }

    std::string odom_topic = "/odom";
    if (sdf->HasElement("odomTopic")) {
        odom_topic = sdf->GetElement("odomTopic")->Get<std::string>();
    }
    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    std::string joint_state_topic = "/joint_states";
    if (sdf->HasElement("jointStateTopic")) {
        joint_state_topic = sdf->GetElement("jointStateTopic")->Get<std::string>();
    }
    joint_state_pub_ =
        ros_node_->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    // Reset timing
    stand_up_start_time_ = model_->GetWorld()->SimTime().Double();
    last_infer_time_ = model_->GetWorld()->SimTime();
    last_publish_time_ = model_->GetWorld()->SimTime();

    // Fill observation history with initial observations
    for (int h = 0; h < HISTORY_LEN; h++) {
        RefreshObservation();
    }

    // Connect to world update event
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&A1RLController::OnUpdate, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(rclcpp::get_logger("a1_rl_controller"),
                       "A1RLController loaded successfully");
}

void A1RLController::LoadPolicy(const std::string& path) {
    ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "A1RLController");

    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    ort_session_ = std::make_unique<Ort::Session>(*ort_env_, path.c_str(), session_options);

    ort_allocator_ = std::make_unique<Ort::AllocatorWithDefaultOptions>();

    Ort::AllocatedStringPtr input_name_ptr =
        ort_session_->GetInputNameAllocated(0, *ort_allocator_);
    input_name_ = std::string(input_name_ptr.get());

    Ort::AllocatedStringPtr output_name_ptr =
        ort_session_->GetOutputNameAllocated(0, *ort_allocator_);
    output_name_ = std::string(output_name_ptr.get());

    // Get input shape from model (ONNX may report dynamic batch dim as -1)
    Ort::TypeInfo type_info = ort_session_->GetInputTypeInfo(0);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    input_shape_ = tensor_info.GetShape();
    if (!input_shape_.empty() && input_shape_[0] < 0) {
        input_shape_[0] = 1;  // fix dynamic batch dim for CreateTensor
    }

    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("a1_rl_controller"),
                       "ONNX model loaded. Input: " << input_name_
                                                    << " [" << input_shape_[0] << ", "
                                                    << input_shape_[1] << "]"
                                                    << ", Output: " << output_name_);
}

void A1RLController::OnUpdate(const gazebo::common::UpdateInfo& info) {
    gazebo::common::Time now = info.simTime;

    // Read joint positions and velocities every step (Gazebo order: FR, FL, RR, RL)
    std::array<float, 12> dof_pos;
    std::array<float, 12> dof_vel;
    for (int i = 0; i < 12; i++) {
        dof_pos[i] = static_cast<float>(joints_[i]->Position(0));
        dof_vel[i] = static_cast<float>(joints_[i]->GetVelocity(0));
    }

    if (state_ == STAND_UP) {
        double stand_elapsed = now.Double() - stand_up_start_time_;
        double percent = stand_elapsed / stand_up_duration_;
        if (percent > 1.0) percent = 1.0;

        // Interpolate from start_pos to default_dof_pos (Gazebo joint order)
        for (int i = 0; i < 12; i++) {
            int rl_idx = reindex_[i];
            float target = (1.0f - static_cast<float>(percent)) * start_pos_[i] +
                           static_cast<float>(percent) * default_dof_pos_[rl_idx];
            target_q_[i] = target;
            float torque = kp_stand_up_ * (target - dof_pos[i]) - kd_stand_up_ * dof_vel[i];
            joints_[i]->SetForce(0, torque);
        }

        if (percent >= 1.0) {
            state_ = RL_RUNNING;
            last_infer_time_ = now;
            RCLCPP_INFO_STREAM(rclcpp::get_logger("a1_rl_controller"),
                               "Stand-up complete, switching to RL_RUNNING");
        }
    } else {
        // RL_RUNNING — throttle inference to 50Hz, apply PD torque every step
        double elapsed = (now - last_infer_time_).Double();
        if (elapsed >= infer_duration_) {
            last_infer_time_ = now;

            if (CheckSafety()) {
                for (int i = 0; i < 12; i++) {
                    target_q_[i] = default_dof_pos_[reindex_[i]];
                }
            } else {
                RefreshObservation();

                std::vector<float> input_data(HISTORY_LEN * OBS_DIM);
                std::memcpy(input_data.data(), obs_history_.data(),
                            HISTORY_LEN * OBS_DIM * sizeof(float));

                std::vector<const char*> input_names = {input_name_.c_str()};
                std::vector<const char*> output_names = {output_name_.c_str()};

                std::vector<Ort::Value> input_tensors;
                input_tensors.push_back(Ort::Value::CreateTensor<float>(
                    memory_info_, input_data.data(), input_data.size(),
                    input_shape_.data(), input_shape_.size()));

                try {
                    auto output_tensors =
                        ort_session_->Run(Ort::RunOptions{nullptr}, input_names.data(),
                                          input_tensors.data(), 1, output_names.data(), 1);

                    float* action_data = output_tensors[0].GetTensorMutableData<float>();
                    std::array<float, 12> actions;
                    std::memcpy(actions.data(), action_data, 12 * sizeof(float));

                    last_actions_ = actions;

                    for (int i = 0; i < 12; i++) {
                        int rl_idx = reindex_[i];
                        target_q_[i] = actions[rl_idx] * 0.25f + default_dof_pos_[rl_idx];
                    }
                } catch (const Ort::Exception& e) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("a1_rl_controller"),
                                        "ONNX inference error: " << e.what());
                }
            }
        }

        // PD torque tracking — EVERY physics step
        for (int i = 0; i < 12; i++) {
            float torque = kp_rl_ * (target_q_[i] - dof_pos[i]) - kd_rl_ * dof_vel[i];
            joints_[i]->SetForce(0, torque);
        }
    }

    // Publish state (throttled)
    double pub_elapsed = (now - last_publish_time_).Double();
    if (pub_elapsed >= publish_duration_) {
        last_publish_time_ = now;
        PublishState(now);
    }
}

void A1RLController::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_linear_x_ = msg->linear.x;
    cmd_linear_y_ = msg->linear.y;
    cmd_angular_z_ = msg->angular.z;
    last_cmd_vel_time_ = model_->GetWorld()->SimTime().Double();
}

void A1RLController::RefreshObservation() {
    // cmd_vel timeout: zero velocity if no message received recently
    double sim_time = model_->GetWorld()->SimTime().Double();
    if (sim_time - last_cmd_vel_time_ > cmd_vel_timeout_) {
        cmd_linear_x_ = 0.0;
        cmd_linear_y_ = 0.0;
        cmd_angular_z_ = 0.0;
    }

    // Get base orientation and angular velocity
    ignition::math::Pose3d world_pose = model_->WorldPose();
    ignition::math::Quaterniond q = world_pose.Rot();  // (w, x, y, z)
    ignition::math::Vector3d world_ang_vel = model_->WorldAngularVel();

    // Rotate world-frame vectors into body frame (inverse rotation)
    ignition::math::Vector3d body_ang_vel = q.RotateVectorReverse(world_ang_vel);
    ignition::math::Vector3d gravity(0, 0, -1.0);
    ignition::math::Vector3d projected_gravity = q.RotateVectorReverse(gravity);

    // Read joint positions in RL model order
    std::array<float, 12> rl_dof_pos;
    std::array<float, 12> rl_dof_vel;
    for (int i = 0; i < 12; i++) {
        rl_dof_pos[reindex_[i]] = static_cast<float>(joints_[i]->Position(0));
        rl_dof_vel[reindex_[i]] = static_cast<float>(joints_[i]->GetVelocity(0));
    }

    // Build 45-dim observation
    std::array<float, OBS_DIM> obs{};
    int idx = 0;

    // [0:3]  body angular velocity * 0.25
    obs[idx++] = static_cast<float>(body_ang_vel.X()) * 0.25f;
    obs[idx++] = static_cast<float>(body_ang_vel.Y()) * 0.25f;
    obs[idx++] = static_cast<float>(body_ang_vel.Z()) * 0.25f;

    // [3:6]  projected gravity
    obs[idx++] = static_cast<float>(projected_gravity.X());
    obs[idx++] = static_cast<float>(projected_gravity.Y());
    obs[idx++] = static_cast<float>(projected_gravity.Z());

    // [6:9]  cmd_vel * [2, 2, 0.25]
    obs[idx++] = static_cast<float>(cmd_linear_x_) * 2.0f;
    obs[idx++] = static_cast<float>(cmd_linear_y_) * 2.0f;
    obs[idx++] = static_cast<float>(cmd_angular_z_) * 0.25f;

    // [9:21] (dof_pos - default_dof_pos) * 1.0
    for (int i = 0; i < 12; i++) {
        obs[idx++] = (rl_dof_pos[i] - default_dof_pos_[i]) * 1.0f;
    }

    // [21:33] dof_vel * 0.05
    for (int i = 0; i < 12; i++) {
        obs[idx++] = rl_dof_vel[i] * 0.05f;
    }

    // [33:45] last_actions (scaled, same as original stores unscaled)
    for (int i = 0; i < 12; i++) {
        obs[idx++] = last_actions_[i];
    }

    // Slide history: shift left by 45, insert new at the end
    std::memmove(obs_history_.data(), obs_history_.data() + OBS_DIM,
                 (HISTORY_LEN - 1) * OBS_DIM * sizeof(float));
    std::memcpy(obs_history_.data() + (HISTORY_LEN - 1) * OBS_DIM, obs.data(),
                OBS_DIM * sizeof(float));
}


void A1RLController::PublishState(const gazebo::common::Time& now) {
    ignition::math::Pose3d world_pose = model_->WorldPose();

    // Odometry in odom frame (relative to initial spawn pose)
    ignition::math::Pose3d rel_pose = world_pose - initial_pose_;

    rclcpp::Time ros_now(now.sec, now.nsec);

    // Publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = ros_now;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = robot_base_frame_;
    odom.pose.pose.position.x = rel_pose.Pos().X();
    odom.pose.pose.position.y = rel_pose.Pos().Y();
    odom.pose.pose.position.z = rel_pose.Pos().Z();
    odom.pose.pose.orientation.w = rel_pose.Rot().W();
    odom.pose.pose.orientation.x = rel_pose.Rot().X();
    odom.pose.pose.orientation.y = rel_pose.Rot().Y();
    odom.pose.pose.orientation.z = rel_pose.Rot().Z();

    ignition::math::Vector3d world_lin_vel = model_->WorldLinearVel();
    ignition::math::Vector3d world_ang_vel = model_->WorldAngularVel();
    odom.twist.twist.linear.x = world_lin_vel.X();
    odom.twist.twist.linear.y = world_lin_vel.Y();
    odom.twist.twist.linear.z = world_lin_vel.Z();
    odom.twist.twist.angular.x = world_ang_vel.X();
    odom.twist.twist.angular.y = world_ang_vel.Y();
    odom.twist.twist.angular.z = world_ang_vel.Z();

    // Set covariance (unknown = 0, use identity for simplicity)
    for (int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = 0.0;
        odom.twist.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[14] = 0.01;

    odom_pub_->publish(odom);

    // Broadcast TF: odom → base_footprint
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = ros_now;
    tf.header.frame_id = odom_frame_id_;
    tf.child_frame_id = robot_base_frame_;
    tf.transform.translation.x = rel_pose.Pos().X();
    tf.transform.translation.y = rel_pose.Pos().Y();
    tf.transform.translation.z = rel_pose.Pos().Z();
    tf.transform.rotation.w = rel_pose.Rot().W();
    tf.transform.rotation.x = rel_pose.Rot().X();
    tf.transform.rotation.y = rel_pose.Rot().Y();
    tf.transform.rotation.z = rel_pose.Rot().Z();
    tf_broadcaster_->sendTransform(tf);

    // Publish joint states
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = ros_now;
    for (int i = 0; i < 12; i++) {
        joint_state.name.push_back(joints_[i]->GetName());
        joint_state.position.push_back(joints_[i]->Position(0));
        joint_state.velocity.push_back(joints_[i]->GetVelocity(0));
        joint_state.effort.push_back(joints_[i]->GetForce(0));
    }
    joint_state_pub_->publish(joint_state);
}

bool A1RLController::CheckSafety() {
    ignition::math::Quaterniond q = model_->WorldPose().Rot();

    // Compute trunk Z-axis tilt (pitch + roll from vertical)
    ignition::math::Vector3d z_axis(0, 0, 1);
    ignition::math::Vector3d trunk_z = q.RotateVector(z_axis);

    // Angle from vertical
    double dot = trunk_z.Dot(z_axis);
    dot = std::max(-1.0, std::min(1.0, dot));
    double tilt = std::acos(dot);

    if (tilt > max_tilt_) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("a1_rl_controller"),
                           "Safety triggered: trunk tilt = " << tilt * 180.0 / M_PI << "°");
        return true;
    }
    return false;
}

GZ_REGISTER_MODEL_PLUGIN(A1RLController)

}  // namespace simulation
