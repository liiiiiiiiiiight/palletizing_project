#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <atomic>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include "kuka_interfaces/action/pallet_task.hpp"

using namespace std::chrono_literals;
using PalletTask = kuka_interfaces::action::PalletTask;
using GoalHandlePalletTask = rclcpp_action::ServerGoalHandle<PalletTask>;

class PalletizingServer : public rclcpp::Node
{
public:
    PalletizingServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("palletizing_server", options)
    {
        suction_pub_ = this->create_publisher<std_msgs::msg::Bool>("/suction_cmd", 10);

        action_server_ = rclcpp_action::create_server<PalletTask>(
            this,
            "pallet_task",
            std::bind(&PalletizingServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PalletizingServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PalletizingServer::handle_accepted, this, std::placeholders::_1));

        execution_thread_ = std::thread(&PalletizingServer::execution_worker, this);

        RCLCPP_INFO(this->get_logger(), "码垛 Action Server 已拉起，等待机械臂初始化...");
    }

    void init() {
        std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "开始初始化 MoveGroup...");
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
            move_group_->setEndEffectorLink("ee");

            // 初始化 planning_scene_monitor
            planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                shared_from_this(), "robot_description");
            planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
            planning_scene_monitor_->startSceneMonitor();
            planning_scene_monitor_->startStateMonitor();

            move_group_->setPlanningTime(5.0);
            move_group_->setNumPlanningAttempts(3);

            // 等待机器人状态可用后再应用4DOF约束
            RCLCPP_INFO(this->get_logger(), "应用4DOF约束...");
            apply_4dof_constraints(10.0);

            RCLCPP_INFO(this->get_logger(), "尝试运动到初始位置...");
            // 跳过运动到home位置的步骤，直接使用当前状态作为起始状态
            // 因为当前状态已经是[0, 0, 0, 0, 0, 0]，符合4DOF约束
            // if (!move_to_home_position()) {
            //     RCLCPP_ERROR(this->get_logger(), "严重错误：无法运动到初始位置，系统挂起！");
            //     return;
            // }
            RCLCPP_INFO(this->get_logger(), "跳过运动到初始位置步骤，直接使用当前状态");

            auto current_state = move_group_->getCurrentState(10.0);

            if (current_state != nullptr) {
                // 确保virtual_future_state也满足4DOF约束
                std::vector<double> jpos;
                current_state->copyJointGroupPositions(PLANNING_GROUP, jpos);
                if (jpos.size() >= 6) {
                    jpos[3] = PASSIVE_JOINT4_POS;
                    jpos[4] = compute_joint5(jpos[1], jpos[2]);
                    current_state->setJointGroupPositions(PLANNING_GROUP, jpos);
                }
                virtual_future_state_ = std::make_shared<moveit::core::RobotState>(*current_state);
                RCLCPP_INFO(this->get_logger(), "MoveGroup 初始化完成！起点状态已同步。流水线准备就绪。");
                system_ready_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "初始化失败：等待 10 秒后仍未获取到机器人的 joint_states！");
            }
        }).detach();
    }

    ~PalletizingServer() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_execution_ = true;
        }
        queue_cv_.notify_all();
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    std::shared_ptr<moveit::core::RobotState> virtual_future_state_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub_;
    rclcpp_action::Server<PalletTask>::SharedPtr action_server_;

    const std::string PLANNING_GROUP = "arm";
    std::atomic<bool> system_ready_{false};
    std::atomic<bool> is_executing_{false};

    // 4DOF配置：J4固定在0度，J5由J2+J3动态决定
    const double PASSIVE_JOINT4_POS = 0.0;
    // J2 + J3 + J5 = π/2 → J5 = π/2 - (J2 + J3)

    double compute_joint5(double j2, double j3) const {
        return M_PI / 2.0 - (j2 + j3);
    }

    // 自定义4DOF IK：在J4=0, J5=π/2-(J2+J3)约束下求解(J1,J2,J3,J6)
    // 使用数值Jacobian迭代，只优化4个自由度
    // 返回6个关节角 [J1,J2,J3,J4,J5,J6]，失败返回空
    std::optional<std::vector<double>> solve_4dof_ik(
        const geometry_msgs::msg::Pose& target,
        const std::vector<double>& seed_joints)
    {
        const int MAX_ITER = 300;       // 增加迭代次数提高收敛率
        const double POS_TOL = 0.002;   // 放宽位置容差到2mm
        const double ROT_TOL = 0.02;    // 放宽旋转容差到~1.15度
        const double EPS = 1e-6;        // 数值Jacobian扰动量
        const double DAMPING = 0.01;    // 减小阻尼因子，提高收敛性

        std::vector<double> joints = seed_joints;
        if (joints.size() < 6) return std::nullopt;

        // J1初始化：用目标位置的atan2
        joints[0] = atan2(target.position.y, target.position.x);
        // J2/J3初始化：使用种子值，但如果种子值不合理则使用默认值
        if (std::abs(joints[1]) > M_PI || std::abs(joints[2]) > M_PI) {
            // 种子值不合理，使用中间值
            joints[1] = -0.5;
            joints[2] = 0.5;
        }
        joints[3] = PASSIVE_JOINT4_POS;
        joints[4] = compute_joint5(joints[1], joints[2]);

        // 提取目标位姿为Eigen格式
        Eigen::Vector3d target_pos(target.position.x, target.position.y, target.position.z);
        Eigen::Quaterniond target_quat(target.orientation.w, target.orientation.x,
                                        target.orientation.y, target.orientation.z);

        const moveit::core::JointModelGroup* jmg =
            move_group_->getRobotModel()->getJointModelGroup(PLANNING_GROUP);

        // 4个自由度的索引：J1=0, J2=1, J3=2, J6=5
        const int active_idx[4] = {0, 1, 2, 5};

        double final_pos_err = 0.0;
        for (int iter = 0; iter < MAX_ITER; ++iter) {
            // 确保J5满足约束
            joints[4] = compute_joint5(joints[1], joints[2]);

            // FK: 计算当前关节角对应的末端位姿
            moveit::core::RobotState test_state(move_group_->getRobotModel());
            test_state.setJointGroupPositions(PLANNING_GROUP, joints);
            test_state.update();
            Eigen::Isometry3d fk_pose = test_state.getGlobalLinkTransform("ee");

            // 位置误差
            Eigen::Vector3d pos_err = target_pos - fk_pose.translation();

            // 旋转误差（用ZYX欧拉角提取yaw）
            Eigen::Quaterniond fk_quat(fk_pose.rotation());
            // 计算旋转误差的yaw分量
            double target_yaw = atan2(2.0 * (target_quat.w() * target_quat.z() +
                                              target_quat.x() * target_quat.y()),
                                       1.0 - 2.0 * (target_quat.y() * target_quat.y() +
                                                     target_quat.z() * target_quat.z()));
            double fk_yaw = atan2(2.0 * (fk_quat.w() * fk_quat.z() +
                                          fk_quat.x() * fk_quat.y()),
                                   1.0 - 2.0 * (fk_quat.y() * fk_quat.y() +
                                                 fk_quat.z() * fk_quat.z()));
            double yaw_err = target_yaw - fk_yaw;
            // 归一化到[-π, π]
            while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

            // 收敛判断
            double pos_norm = pos_err.norm();
            if (pos_norm < POS_TOL && std::abs(yaw_err) < ROT_TOL) {
                RCLCPP_DEBUG(this->get_logger(), "4DOF IK收敛: iter=%d, pos_err=%.4f, yaw_err=%.4f",
                            iter, pos_norm, yaw_err);
                return joints;
            }
            final_pos_err = pos_norm;

            // 构建误差向量 [dx, dy, dz, dyaw]
            Eigen::Vector4d err;
            err << pos_err, yaw_err;

            // 数值Jacobian：4个主动关节 × 4个误差分量
            Eigen::Matrix4d J = Eigen::Matrix4d::Zero();
            for (int col = 0; col < 4; ++col) {
                int idx = active_idx[col];
                std::vector<double> joints_plus = joints;
                joints_plus[idx] += EPS;
                joints_plus[4] = compute_joint5(joints_plus[1], joints_plus[2]);

                moveit::core::RobotState state_plus(move_group_->getRobotModel());
                state_plus.setJointGroupPositions(PLANNING_GROUP, joints_plus);
                state_plus.update();
                Eigen::Isometry3d fk_plus = state_plus.getGlobalLinkTransform("ee");

                Eigen::Vector3d dpos = (fk_plus.translation() - fk_pose.translation()) / EPS;

                Eigen::Quaterniond q_plus(fk_plus.rotation());
                double yaw_plus = atan2(2.0 * (q_plus.w() * q_plus.z() + q_plus.x() * q_plus.y()),
                                         1.0 - 2.0 * (q_plus.y() * q_plus.y() + q_plus.z() * q_plus.z()));
                double dyaw = (yaw_plus - fk_yaw) / EPS;

                J.col(col) << dpos, dyaw;
            }

            // 阻尼最小二乘求解：Δq = J^T (J J^T + λ²I)^{-1} e
            Eigen::Matrix4d JJt = J * J.transpose() + DAMPING * DAMPING * Eigen::Matrix4d::Identity();
            Eigen::Vector4d dq = J.transpose() * JJt.ldlt().solve(err);

            // 步长限制，防止过大跳变
            double max_step = 0.3;  // 增加最大单步到0.3 rad
            double step_norm = dq.norm();
            if (step_norm > max_step) {
                dq *= max_step / step_norm;
            }

            // 更新主动关节
            for (int col = 0; col < 4; ++col) {
                joints[active_idx[col]] += dq(col);
            }

            // 关节限位钳位
            const std::vector<std::string>& joint_names = jmg->getJointModelNames();
            for (int col = 0; col < 4; ++col) {
                int idx = active_idx[col];
                const moveit::core::JointModel* jm = jmg->getJointModel(joint_names[idx]);
                if (jm && jm->getVariableCount() == 1) {
                    const auto& bounds = jm->getVariableBounds()[0];
                    joints[idx] = std::max(bounds.min_position_, std::min(bounds.max_position_, joints[idx]));
                }
            }
        }

        RCLCPP_WARN(this->get_logger(), "4DOF IK未收敛 (pos_err=%.4f)", final_pos_err);
        return std::nullopt;
    }

    // 自定义4DOF IK：在J4=0, J5=π/2-(J2+J3)约束下求解(J1,J2,J3,J6)
    // 使用数值Jacobian迭代，只优化4个自由度
    // 返回6个关节角 [J1,J2,J3,J4,J5,J6]，失败返回空
    // 多起点版本：尝试多个不同的初始J2/J3值，提高收敛率
    std::optional<std::vector<double>> solve_4dof_ik_multi_start(
        const geometry_msgs::msg::Pose& target,
        const std::vector<double>& seed_joints)
    {
        const int MAX_ITER = 500;       // 增加迭代次数提高收敛率
        const double POS_TOL = 0.0015;  // 位置容差
        const double ROT_TOL = 0.015;   // 旋转容差
        const double EPS = 1e-6;        // 数值Jacobian扰动量
        const double DAMPING = 0.02;    // 阻尼因子

        // 多起点策略：使用不同的初始J2,J3值
        const int NUM_STARTS = 8;
        const double START_J2_OFFSETS[NUM_STARTS] = {0.0, -0.3, 0.3, -0.6, 0.6, -0.15, 0.15, -0.45};
        const double START_J3_OFFSETS[NUM_STARTS] = {0.0, 0.3, -0.3, 0.6, -0.6, 0.15, -0.15, 0.45};

        std::optional<std::vector<double>> best_solution;
        double best_error = std::numeric_limits<double>::max();

        for (int start_idx = 0; start_idx < NUM_STARTS; ++start_idx) {
            std::vector<double> joints = seed_joints;
            if (joints.size() < 6) continue;

            // J1初始化：用目标位置的atan2
            joints[0] = atan2(target.position.y, target.position.x);
            joints[3] = PASSIVE_JOINT4_POS;
            // 使用不同的J2/J3初始值，但先检查种子值是否合理
            double j2_init = seed_joints[1];
            double j3_init = seed_joints[2];
            // 如果种子值不合理，使用默认值
            if (std::abs(j2_init) > M_PI || std::abs(j3_init) > M_PI) {
                j2_init = -0.5;
                j3_init = 0.5;
            }
            joints[1] = j2_init + START_J2_OFFSETS[start_idx];
            joints[2] = j3_init + START_J3_OFFSETS[start_idx];
            joints[4] = compute_joint5(joints[1], joints[2]);

            // 提取目标位姿为Eigen格式
            Eigen::Vector3d target_pos(target.position.x, target.position.y, target.position.z);
            Eigen::Quaterniond target_quat(target.orientation.w, target.orientation.x,
                                            target.orientation.y, target.orientation.z);

            const moveit::core::JointModelGroup* jmg =
                move_group_->getRobotModel()->getJointModelGroup(PLANNING_GROUP);

            // 4个自由度的索引：J1=0, J2=1, J3=2, J6=5
            const int active_idx[4] = {0, 1, 2, 5};

            bool converged = false;
            double final_pos_err = 0.0;

            for (int iter = 0; iter < MAX_ITER; ++iter) {
                // 确保J5满足约束
                joints[4] = compute_joint5(joints[1], joints[2]);

                // FK: 计算当前关节角对应的末端位姿
                moveit::core::RobotState test_state(move_group_->getRobotModel());
                test_state.setJointGroupPositions(PLANNING_GROUP, joints);
                test_state.update();
                Eigen::Isometry3d fk_pose = test_state.getGlobalLinkTransform("ee");

                // 位置误差
                Eigen::Vector3d pos_err = target_pos - fk_pose.translation();

                // 旋转误差（用ZYX欧拉角提取yaw）
                Eigen::Quaterniond fk_quat(fk_pose.rotation());
                double target_yaw = atan2(2.0 * (target_quat.w() * target_quat.z() +
                                                  target_quat.x() * target_quat.y()),
                                           1.0 - 2.0 * (target_quat.y() * target_quat.y() +
                                                         target_quat.z() * target_quat.z()));
                double fk_yaw = atan2(2.0 * (fk_quat.w() * fk_quat.z() +
                                              fk_quat.x() * fk_quat.y()),
                                       1.0 - 2.0 * (fk_quat.y() * fk_quat.y() +
                                                     fk_quat.z() * fk_quat.z()));
                double yaw_err = target_yaw - fk_yaw;
                while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
                while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

                // 收敛判断
                double pos_norm = pos_err.norm();
                if (pos_norm < POS_TOL && std::abs(yaw_err) < ROT_TOL) {
                    RCLCPP_DEBUG(this->get_logger(), "4DOF IK收敛: start=%d, iter=%d, pos_err=%.4f, yaw_err=%.4f",
                                start_idx, iter, pos_norm, yaw_err);
                    converged = true;
                    if (pos_norm < best_error) {
                        best_error = pos_norm;
                        best_solution = joints;
                    }
                    break;
                }
                final_pos_err = pos_norm;

                // 构建误差向量 [dx, dy, dz, dyaw]
                Eigen::Vector4d err;
                err << pos_err, yaw_err;

                // 数值Jacobian：4个主动关节 × 4个误差分量
                Eigen::Matrix4d J = Eigen::Matrix4d::Zero();
                for (int col = 0; col < 4; ++col) {
                    int idx = active_idx[col];
                    std::vector<double> joints_plus = joints;
                    joints_plus[idx] += EPS;
                    joints_plus[4] = compute_joint5(joints_plus[1], joints_plus[2]);

                    moveit::core::RobotState state_plus(move_group_->getRobotModel());
                    state_plus.setJointGroupPositions(PLANNING_GROUP, joints_plus);
                    state_plus.update();
                    Eigen::Isometry3d fk_plus = state_plus.getGlobalLinkTransform("ee");

                    Eigen::Vector3d dpos = (fk_plus.translation() - fk_pose.translation()) / EPS;

                    Eigen::Quaterniond q_plus(fk_plus.rotation());
                    double yaw_plus = atan2(2.0 * (q_plus.w() * q_plus.z() + q_plus.x() * q_plus.y()),
                                             1.0 - 2.0 * (q_plus.y() * q_plus.y() + q_plus.z() * q_plus.z()));
                    double dyaw = (yaw_plus - fk_yaw) / EPS;

                    J.col(col) << dpos, dyaw;
                }

                // 阻尼最小二乘求解
                Eigen::Matrix4d JJt = J * J.transpose() + DAMPING * DAMPING * Eigen::Matrix4d::Identity();
                Eigen::Vector4d dq = J.transpose() * JJt.ldlt().solve(err);

                // 步长限制
                double max_step = 0.25;
                double step_norm = dq.norm();
                if (step_norm > max_step) {
                    dq *= max_step / step_norm;
                }

                // 更新主动关节
                for (int col = 0; col < 4; ++col) {
                    joints[active_idx[col]] += dq(col);
                }

                // 关节限位钳位
                const std::vector<std::string>& joint_names = jmg->getJointModelNames();
                for (int col = 0; col < 4; ++col) {
                    int idx = active_idx[col];
                    const moveit::core::JointModel* jm = jmg->getJointModel(joint_names[idx]);
                    if (jm && jm->getVariableCount() == 1) {
                        const auto& bounds = jm->getVariableBounds()[0];
                        joints[idx] = std::max(bounds.min_position_, std::min(bounds.max_position_, joints[idx]));
                    }
                }
            }

            // 如果这个起点收敛了，返回结果
            if (converged && best_solution.has_value()) {
                return best_solution;
            }
        }

        if (!best_solution.has_value()) {
            RCLCPP_WARN(this->get_logger(), "4DOF IK多起点未收敛 (best_error=%.4f)", best_error);
        }
        return best_solution;
    }

    // 轨迹结构体：3段式码垛路径
    struct ExecutionTask {
        moveit_msgs::msg::RobotTrajectory approach_traj;   // current → pick (含直线下降)
        moveit_msgs::msg::RobotTrajectory place_traj;      // pick → place (含直线上升+转移+直线下降)
        moveit_msgs::msg::RobotTrajectory retract_traj;    // place → place_above (含直线上升)
        std::shared_ptr<GoalHandlePalletTask> goal_handle;
    };

    std::queue<ExecutionTask> task_queue_;
    std::mutex queue_mutex_;
    std::mutex planning_mutex_;
    std::condition_variable queue_cv_;
    bool stop_execution_ = false;
    std::thread execution_thread_;

    bool execute_with_alignment(moveit::planning_interface::MoveGroupInterface::Plan& plan, const std::string& phase_name) {
        if (plan.trajectory_.joint_trajectory.points.empty()) return false;

        auto current_state = move_group_->getCurrentState(1.0);
        if (!current_state) return false;

        std::vector<double> current_positions;
        current_state->copyJointGroupPositions(PLANNING_GROUP, current_positions);
        auto& start_positions = plan.trajectory_.joint_trajectory.points[0].positions;

        if (current_positions.size() != start_positions.size()) return false;

        double max_dev = 0.0;
        for (size_t i = 0; i < current_positions.size(); ++i) {
            if (i == 3) continue;
            double dev = std::abs(current_positions[i] - start_positions[i]);
            if (dev > max_dev) max_dev = dev;
        }

        if (max_dev > 0.005 && max_dev < 0.1) {
            plan.trajectory_.joint_trajectory.points[0].positions = current_positions;
        } else if (max_dev >= 0.1) {
            RCLCPP_ERROR(this->get_logger(), "[%s] 物理位置严重偏离预设轨迹起点 (偏差: %.4f rad)，拒绝执行！", phase_name.c_str(), max_dev);
            return false;
        }

        plan.trajectory_.joint_trajectory.header.stamp = this->now();
        return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    // 应用4DOF约束：J4固定为0，J5动态计算为π/2-(J2+J3)
    void apply_4dof_constraints(double timeout = 0.0) {
        if (!move_group_) return;
        auto current_state = move_group_->getCurrentState(timeout);
        if (!current_state) return;

        std::vector<double> joint_positions;
        current_state->copyJointGroupPositions(PLANNING_GROUP, joint_positions);
        if (joint_positions.size() >= 6) {
            joint_positions[3] = PASSIVE_JOINT4_POS;
            joint_positions[4] = compute_joint5(joint_positions[1], joint_positions[2]);
        }
        current_state->setJointGroupPositions(PLANNING_GROUP, joint_positions);
        move_group_->setStartState(*current_state);
    }

    bool move_to_home_position() {
        // 使用与initial_positions.yaml一致的home位置
        // initial_positions: joint1=0, joint2=-1.570796, joint3=1.570796, joint4=0, joint5=1.570796, joint6=0
        double home_j2 = -90.0 * M_PI / 180.0;
        double home_j3 = 90.0 * M_PI / 180.0;
        std::vector<double> home_joints = {
            0.0, home_j2, home_j3,
            PASSIVE_JOINT4_POS, compute_joint5(home_j2, home_j3), 0.0
        };

        auto start_state = move_group_->getCurrentState(10.0);
        if (!start_state) return false;
        {
            std::vector<double> start_joints;
            start_state->copyJointGroupPositions(PLANNING_GROUP, start_joints);
            if (start_joints.size() >= 6) {
                start_joints[3] = PASSIVE_JOINT4_POS;
                start_joints[4] = compute_joint5(start_joints[1], start_joints[2]);
                start_state->setJointGroupPositions(PLANNING_GROUP, start_joints);
            }
            move_group_->setStartState(*start_state);
        }

        moveit::core::RobotState target_state(move_group_->getRobotModel());
        target_state.setJointGroupPositions(PLANNING_GROUP, home_joints);
        move_group_->setJointValueTarget(target_state);

        // 尝试多种planner配置
        std::vector<std::pair<std::string, std::string>> planner_configs = {
            {"ompl", "RRTConnectkConfigDefault"},
            {"pilz_industrial_motion_planner", "PTP"},
            {"pilz_industrial_motion_planner", "LIN"}
        };

        for (const auto& config : planner_configs) {
            RCLCPP_INFO(this->get_logger(), "尝试使用 %s/%s 运动到初始位置...",
                       config.first.c_str(), config.second.c_str());

            move_group_->setPlanningPipelineId(config.first);
            move_group_->setPlannerId(config.second);
            move_group_->setPlanningTime(5.0);
            move_group_->setNumPlanningAttempts(3);
            move_group_->setJointValueTarget(target_state);

            auto move_result = move_group_->move();
            if (move_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "使用 %s/%s 成功运动到初始位置",
                           config.first.c_str(), config.second.c_str());
                return true;
            }
        }

        RCLCPP_ERROR(this->get_logger(), "所有planner都无法运动到初始位置");
        return false;
    }

    // ===== 检查轨迹是否合法（无碰撞） =====
    bool validate_trajectory_collisions(const robot_trajectory::RobotTrajectory& traj) {
        if (traj.getWayPointCount() == 0) {
            RCLCPP_DEBUG(this->get_logger(), "空轨迹，视为无效");
            return false;
        }

        // 使用 planning_scene_monitor 获取当前的 planning scene
        if (!planning_scene_monitor_) {
            RCLCPP_DEBUG(this->get_logger(), "planning_scene_monitor 未初始化，跳过碰撞验证");
            return true;
        }

        auto planning_scene = planning_scene_monitor_->getPlanningScene();
        if (!planning_scene) {
            RCLCPP_DEBUG(this->get_logger(), "无法获取 planning scene，跳过碰撞验证");
            return true;
        }

        // 检查轨迹中每个点的碰撞状态
        for (size_t i = 0; i < traj.getWayPointCount(); ++i) {
            const auto& waypoint = traj.getWayPoint(i);
            if (planning_scene->isStateColliding(waypoint, PLANNING_GROUP)) {
                RCLCPP_WARN(this->get_logger(), "轨迹验证失败: 在索引 %zu 处检测到碰撞", i);
                return false;
            }
        }

        // 检查整个路径的连续碰撞（路径碰撞检查）
        if (!planning_scene->isPathValid(traj, PLANNING_GROUP)) {
            RCLCPP_WARN(this->get_logger(), "轨迹验证失败: 路径段存在碰撞");
            return false;
        }

        RCLCPP_DEBUG(this->get_logger(), "轨迹验证通过: 共 %zu 个点，无碰撞", traj.getWayPointCount());
        return true;
    }

    // ===== OMPL规划（用于转移段，自动避障） =====
    bool plan_ompl_segment(
        const std::vector<double>& start_joints,
        const std::vector<double>& target_joints,
        robot_trajectory::RobotTrajectory& out_traj,
        double planning_time = 5.0)
    {
        // 设置起始状态
        moveit::core::RobotState start_state(move_group_->getRobotModel());
        start_state.setJointGroupPositions(PLANNING_GROUP, start_joints);
        start_state.update();
        move_group_->setStartState(start_state);

        // 设置目标状态
        moveit::core::RobotState target_state(move_group_->getRobotModel());
        target_state.setJointGroupPositions(PLANNING_GROUP, target_joints);
        move_group_->setJointValueTarget(target_state);

        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setPlanningTime(planning_time);
        move_group_->setNumPlanningAttempts(5);  // 增加规划尝试次数
        move_group_->clearPathConstraints();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            RCLCPP_WARN(this->get_logger(), "OMPL规划失败: start=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f], target=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                       start_joints[0], start_joints[1], start_joints[2], start_joints[3], start_joints[4], start_joints[5],
                       target_joints[0], target_joints[1], target_joints[2], target_joints[3], target_joints[4], target_joints[5]);
        }
        if (success && !plan.trajectory_.joint_trajectory.points.empty()) {
            enforce_4dof_in_trajectory(plan.trajectory_);
            moveit::core::RobotState ref_state(move_group_->getRobotModel());
            ref_state.setJointGroupPositions(PLANNING_GROUP, start_joints);
            out_traj.setRobotTrajectoryMsg(ref_state, plan.trajectory_);

            // 验证轨迹无碰撞
            if (!validate_trajectory_collisions(out_traj)) {
                RCLCPP_WARN(this->get_logger(), "OMPL规划的轨迹包含碰撞，拒绝使用");
                return false;
            }
        }
        return success;
    }

    // ===== LIN直线段规划（抓取/放置的垂直下降/上升） =====
    bool plan_lin_segment(
        const std::vector<double>& start_joints,
        const std::vector<double>& end_joints,
        robot_trajectory::RobotTrajectory& out_traj)
    {
        moveit::core::RobotState start_state(move_group_->getRobotModel());
        start_state.setJointGroupPositions(PLANNING_GROUP, start_joints);
        start_state.update();
        move_group_->setStartState(start_state);

        moveit::core::RobotState target_state(move_group_->getRobotModel());
        target_state.setJointGroupPositions(PLANNING_GROUP, end_joints);
        move_group_->setJointValueTarget(target_state);

        move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_->setPlannerId("LIN");
        move_group_->setPlanningTime(5.0);  // 增加规划时间以确保路径有效
        move_group_->setNumPlanningAttempts(3);  // 增加尝试次数

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            RCLCPP_WARN(this->get_logger(), "LIN规划失败: start=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f], end=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                       start_joints[0], start_joints[1], start_joints[2], start_joints[3], start_joints[4], start_joints[5],
                       end_joints[0], end_joints[1], end_joints[2], end_joints[3], end_joints[4], end_joints[5]);
        }
        if (success && !plan.trajectory_.joint_trajectory.points.empty()) {
            enforce_4dof_in_trajectory(plan.trajectory_);
            moveit::core::RobotState ref_state(move_group_->getRobotModel());
            ref_state.setJointGroupPositions(PLANNING_GROUP, start_joints);
            out_traj.setRobotTrajectoryMsg(ref_state, plan.trajectory_);

            // 验证轨迹无碰撞
            if (!validate_trajectory_collisions(out_traj)) {
                RCLCPP_WARN(this->get_logger(), "LIN规划的轨迹包含碰撞，拒绝使用");
                return false;
            }
        }
        return success;
    }

    // ===== PTP规划（回退方案） =====
    bool plan_ptp_segment(
        const std::vector<double>& start_joints,
        const std::vector<double>& target_joints,
        robot_trajectory::RobotTrajectory& out_traj)
    {
        moveit::core::RobotState start_state(move_group_->getRobotModel());
        start_state.setJointGroupPositions(PLANNING_GROUP, start_joints);
        start_state.update();
        move_group_->setStartState(start_state);

        moveit::core::RobotState target_state(move_group_->getRobotModel());
        target_state.setJointGroupPositions(PLANNING_GROUP, target_joints);
        move_group_->setJointValueTarget(target_state);

        move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_->setPlannerId("PTP");
        move_group_->setPlanningTime(5.0);
        move_group_->setNumPlanningAttempts(5);  // 增加规划尝试次数
        move_group_->clearPathConstraints();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            RCLCPP_WARN(this->get_logger(), "PTP规划失败: start=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f], target=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                       start_joints[0], start_joints[1], start_joints[2], start_joints[3], start_joints[4], start_joints[5],
                       target_joints[0], target_joints[1], target_joints[2], target_joints[3], target_joints[4], target_joints[5]);
        }
        if (success && !plan.trajectory_.joint_trajectory.points.empty()) {
            enforce_4dof_in_trajectory(plan.trajectory_);
            moveit::core::RobotState ref_state(move_group_->getRobotModel());
            ref_state.setJointGroupPositions(PLANNING_GROUP, start_joints);
            out_traj.setRobotTrajectoryMsg(ref_state, plan.trajectory_);

            // 验证轨迹无碰撞
            if (!validate_trajectory_collisions(out_traj)) {
                RCLCPP_WARN(this->get_logger(), "PTP规划的轨迹包含碰撞，拒绝使用");
                return false;
            }
        }
        return success;
    }

    // 强制轨迹中的4DOF约束：J4固定为0，J5动态计算
    void enforce_4dof_in_trajectory(moveit_msgs::msg::RobotTrajectory& traj, bool clear_vel_acc = false) {
        for (auto& point : traj.joint_trajectory.points) {
            if (point.positions.size() >= 6) {
                point.positions[3] = PASSIVE_JOINT4_POS;
                point.positions[4] = compute_joint5(point.positions[1], point.positions[2]);
            }
            if (clear_vel_acc) {
                point.velocities.assign(point.velocities.size(), 0.0);
                point.accelerations.assign(point.accelerations.size(), 0.0);
            } else {
                if (point.velocities.size() >= 6) {
                    point.velocities[3] = 0.0;
                    point.velocities[4] = -(point.velocities[1] + point.velocities[2]);
                }
                if (point.accelerations.size() >= 6) {
                    point.accelerations[3] = 0.0;
                    point.accelerations[4] = -(point.accelerations[1] + point.accelerations[2]);
                }
            }
        }
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const PalletTask::Goal>) {
        if (!system_ready_) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePalletTask>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePalletTask> goal_handle) {
        std::thread{ [this, goal_handle]() {
            std::lock_guard<std::mutex> plan_lock(planning_mutex_);
            const auto goal = goal_handle->get_goal();

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (task_queue_.empty() && !is_executing_ && system_ready_) {
                    auto current_state = move_group_->getCurrentState(2.0);
                    if (current_state) *virtual_future_state_ = *current_state;
                }
            }

            double box_height = goal->box_size.z;

            geometry_msgs::msg::Pose pick_above = goal->start_pose;
            pick_above.position.z += box_height + 0.05;
            geometry_msgs::msg::Pose pick_pose = goal->start_pose;

            geometry_msgs::msg::Pose place_above = goal->end_pose;
            place_above.position.z += box_height + 0.05;
            geometry_msgs::msg::Pose place_pose = goal->end_pose;
            place_pose.position.z += 0.005;

            // 逐段规划，每段独立TOTG
            // Phase1 (approach): current → pick_above [OMPL转移] → pick [LIN下降]
            // Phase2 (place):   pick → pick_above [LIN上升] → place_above [OMPL转移] → place [LIN下降]
            // Phase3 (retract): place → place_above [LIN上升]

            auto working_state = std::make_shared<moveit::core::RobotState>(*virtual_future_state_);
            bool ok = true;

            // --- Phase1: current → pick_above (OMPL转移) ---
            robot_trajectory::RobotTrajectory approach_traj(move_group_->getRobotModel(), PLANNING_GROUP);
            {
                std::vector<double> start_joints;
                working_state->copyJointGroupPositions(PLANNING_GROUP, start_joints);
                if (start_joints.size() >= 6) {
                    start_joints[3] = PASSIVE_JOINT4_POS;
                    start_joints[4] = compute_joint5(start_joints[1], start_joints[2]);
                }
                auto ik = solve_4dof_ik_multi_start(pick_above, start_joints);
                if (!ik) { ok = false; RCLCPP_WARN(this->get_logger(), "IK失败: pick_above"); }
                else {
                    bool approach_ok = false;
                    // 增加重试次数和timeout递增范围
                    for (double timeout : {2.0, 3.0, 5.0, 8.0, 10.0}) {
                        RCLCPP_INFO(this->get_logger(), "approach OMPL规划 (timeout=%.1fs)...", timeout);
                        if (plan_ompl_segment(start_joints, *ik, approach_traj, timeout)) {
                            approach_ok = true;
                            break;
                        }
                    }
                    if (!approach_ok) {
                        RCLCPP_WARN(this->get_logger(), "approach OMPL失败，尝试PTP");
                        approach_ok = plan_ptp_segment(start_joints, *ik, approach_traj);
                    }
                    if (!approach_ok) { ok = false; RCLCPP_ERROR(this->get_logger(), "approach段规划失败"); }
                    else { working_state->setJointGroupPositions(PLANNING_GROUP, *ik); }
                }
            }

            // --- Phase1续: pick_above → pick (LIN下降) ---
            robot_trajectory::RobotTrajectory descend_pick_traj(move_group_->getRobotModel(), PLANNING_GROUP);
            if (ok) {
                std::vector<double> above_joints;
                working_state->copyJointGroupPositions(PLANNING_GROUP, above_joints);
                auto ik = solve_4dof_ik(pick_pose, above_joints);
                if (!ik) { ok = false; RCLCPP_WARN(this->get_logger(), "IK失败: pick"); }
                else if (!plan_lin_segment(above_joints, *ik, descend_pick_traj)) { ok = false; }
                else { working_state->setJointGroupPositions(PLANNING_GROUP, *ik); }
            }

            // --- Phase2a: pick → pick_above (LIN上升) ---
            robot_trajectory::RobotTrajectory ascend_pick_traj(move_group_->getRobotModel(), PLANNING_GROUP);
            std::vector<double> pick_above_joints;
            if (ok) {
                std::vector<double> pick_joints;
                working_state->copyJointGroupPositions(PLANNING_GROUP, pick_joints);
                auto ik = solve_4dof_ik(pick_above, pick_joints);
                if (!ik) { ok = false; }
                else {
                    pick_above_joints = *ik;
                    if (!plan_lin_segment(pick_joints, *ik, ascend_pick_traj)) { ok = false; }
                    else { working_state->setJointGroupPositions(PLANNING_GROUP, *ik); }
                }
            }

            // --- Phase2b: pick_above → place_above (OMPL转移) ---
            robot_trajectory::RobotTrajectory transit_traj(move_group_->getRobotModel(), PLANNING_GROUP);
            std::vector<double> place_above_joints;
            if (ok) {
                std::vector<double> start_j;
                working_state->copyJointGroupPositions(PLANNING_GROUP, start_j);
                auto ik = solve_4dof_ik(place_above, start_j);
                if (!ik) { ok = false; RCLCPP_WARN(this->get_logger(), "IK失败: place_above"); }
                else {
                    place_above_joints = *ik;
                    // 增加重试次数和timeout递增范围
                    bool transit_ok = false;
                    for (double timeout : {2.0, 3.0, 5.0, 8.0, 10.0}) {
                        RCLCPP_INFO(this->get_logger(), "transit OMPL规划 (timeout=%.1fs)...", timeout);
                        if (plan_ompl_segment(start_j, *ik, transit_traj, timeout)) {
                            transit_ok = true;
                            break;
                        }
                    }
                    if (!transit_ok) {
                        RCLCPP_WARN(this->get_logger(), "transit OMPL失败，尝试PTP");
                        transit_ok = plan_ptp_segment(start_j, *ik, transit_traj);
                    }
                    if (!transit_ok) { ok = false; RCLCPP_ERROR(this->get_logger(), "transit段规划失败"); }
                    else { working_state->setJointGroupPositions(PLANNING_GROUP, *ik); }
                }
            }

            // --- Phase2c: place_above → place (LIN下降) ---
            robot_trajectory::RobotTrajectory descend_place_traj(move_group_->getRobotModel(), PLANNING_GROUP);
            if (ok) {
                std::vector<double> above_j;
                working_state->copyJointGroupPositions(PLANNING_GROUP, above_j);
                auto ik = solve_4dof_ik(place_pose, above_j);
                if (!ik) { ok = false; }
                else if (!plan_lin_segment(above_j, *ik, descend_place_traj)) { ok = false; }
                else { working_state->setJointGroupPositions(PLANNING_GROUP, *ik); }
            }

            // --- Phase3: place → place_above (LIN上升) ---
            robot_trajectory::RobotTrajectory ascend_place_traj(move_group_->getRobotModel(), PLANNING_GROUP);
            if (ok) {
                std::vector<double> place_j;
                working_state->copyJointGroupPositions(PLANNING_GROUP, place_j);
                if (!plan_lin_segment(place_j, place_above_joints, ascend_place_traj)) { ok = false; }
                else { working_state->setJointGroupPositions(PLANNING_GROUP, place_above_joints); }
            }

            if (!ok) {
                auto result = std::make_shared<PalletTask::Result>();
                result->success = false;
                result->message = "轨迹规划失败";
                goal_handle->abort(result);
                return;
            }

            // 合并为3段执行轨迹，每段独立TOTG
            // TOTG (TimeOptimalTrajectoryGeneration) 本身是时间最优算法
            // 不使用 v_scale/a_scale 缩放因子，让算法自动计算最短时间轨迹
            auto traj_to_msg = [&](robot_trajectory::RobotTrajectory& t) -> moveit_msgs::msg::RobotTrajectory {
                if (t.getWayPointCount() < 2) return moveit_msgs::msg::RobotTrajectory();
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                // 使用默认参数，让 TOTG 计算时间最优轨迹
                // 参数: trajectory, max_velocity_scale=1.0, max_acceleration_scale=1.0
                totg.computeTimeStamps(t, 1.0, 1.0);
                moveit_msgs::msg::RobotTrajectory msg;
                t.getRobotTrajectoryMsg(msg);
                enforce_4dof_in_trajectory(msg);
                return msg;
            };

            // Phase1: approach + descend_pick
            for (int i = 0; i < static_cast<int>(descend_pick_traj.getWayPointCount()); ++i)
                approach_traj.addSuffixWayPoint(descend_pick_traj.getWayPoint(i), 0.0);
            auto approach_msg = traj_to_msg(approach_traj);

            // Phase2: ascend_pick + transit + descend_place
            robot_trajectory::RobotTrajectory place_full(move_group_->getRobotModel(), PLANNING_GROUP);
            for (int i = 0; i < static_cast<int>(ascend_pick_traj.getWayPointCount()); ++i)
                place_full.addSuffixWayPoint(ascend_pick_traj.getWayPoint(i), 0.0);
            for (int i = 1; i < static_cast<int>(transit_traj.getWayPointCount()); ++i)
                place_full.addSuffixWayPoint(transit_traj.getWayPoint(i), 0.0);
            for (int i = 1; i < static_cast<int>(descend_place_traj.getWayPointCount()); ++i)
                place_full.addSuffixWayPoint(descend_place_traj.getWayPoint(i), 0.0);
            auto place_msg = traj_to_msg(place_full);

            // Phase3: ascend_place
            auto retract_msg = traj_to_msg(ascend_place_traj);

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                task_queue_.push({approach_msg, place_msg, retract_msg, goal_handle});
                queue_cv_.notify_one();
            }
            *virtual_future_state_ = *working_state;
            RCLCPP_INFO(this->get_logger(), ">>> 路径规划完成 (箱子: %s, approach:%zu place:%zu retract:%zu) <<<",
                        goal->box_id.c_str(),
                        approach_msg.joint_trajectory.points.size(),
                        place_msg.joint_trajectory.points.size(),
                        retract_msg.joint_trajectory.points.size());
        }}.detach();
    }

    void execution_worker() {
        while (rclcpp::ok()) {
            ExecutionTask current_task;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this]() { return !task_queue_.empty() || stop_execution_; });
                if (stop_execution_) return;

                current_task = task_queue_.front();
                task_queue_.pop();
                is_executing_ = true;
            }

            const auto goal = current_task.goal_handle->get_goal();
            auto feedback = std::make_shared<PalletTask::Feedback>();

            // Phase 1: 接近并抓取
            feedback->phase_description = "接近并抓取: " + goal->box_id;
            current_task.goal_handle->publish_feedback(feedback);

            moveit::planning_interface::MoveGroupInterface::Plan plan1;
            plan1.trajectory_ = current_task.approach_traj;
            int max_retries = 5;  // 增加重试次数到5次
            int retry_count = 0;
            while (rclcpp::ok() && !stop_execution_) {
                if (execute_with_alignment(plan1, "Approach")) break;
                retry_count++;
                if (retry_count <= max_retries) {
                    RCLCPP_WARN(this->get_logger(), "接近轨迹执行失败 (重试 %d/%d)，3秒后重试...", retry_count, max_retries);
                    std::this_thread::sleep_for(3000ms);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "接近轨迹执行失败，已达到最大重试次数");
                    break;
                }
            }

            set_suction(true);
            if (!goal->box_id.empty()) move_group_->attachObject(goal->box_id, "ee");
            std::this_thread::sleep_for(200ms);

            // Phase 2: 转移并放置
            feedback->phase_description = "转移并放置: " + goal->box_id;
            current_task.goal_handle->publish_feedback(feedback);

            moveit::planning_interface::MoveGroupInterface::Plan plan2;
            plan2.trajectory_ = current_task.place_traj;
            max_retries = 5;  // 增加重试次数到5次
            retry_count = 0;
            while (rclcpp::ok() && !stop_execution_) {
                if (execute_with_alignment(plan2, "Place")) break;
                retry_count++;
                if (retry_count <= max_retries) {
                    RCLCPP_WARN(this->get_logger(), "放置轨迹执行失败 (重试 %d/%d)，3秒后重试...", retry_count, max_retries);
                    std::this_thread::sleep_for(3000ms);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "放置轨迹执行失败，已达到最大重试次数");
                    break;
                }
            }

            set_suction(false);
            if (!goal->box_id.empty()) move_group_->detachObject(goal->box_id);
            std::this_thread::sleep_for(200ms);

            // Phase 3: 回退
            feedback->phase_description = "抬升回退: " + goal->box_id;
            current_task.goal_handle->publish_feedback(feedback);

            moveit::planning_interface::MoveGroupInterface::Plan plan3;
            plan3.trajectory_ = current_task.retract_traj;
            max_retries = 5;  // 增加重试次数到5次
            retry_count = 0;
            while (rclcpp::ok() && !stop_execution_) {
                if (execute_with_alignment(plan3, "Retract")) break;
                retry_count++;
                if (retry_count <= max_retries) {
                    RCLCPP_WARN(this->get_logger(), "回退轨迹执行失败 (重试 %d/%d)，3秒后重试...", retry_count, max_retries);
                    std::this_thread::sleep_for(3000ms);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "回退轨迹执行失败，已达到最大重试次数");
                    break;
                }
            }

            auto result = std::make_shared<PalletTask::Result>();
            result->success = true;
            result->message = "任务执行成功: " + goal->box_id;
            current_task.goal_handle->succeed(result);

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                is_executing_ = false;
            }
        }
    }

    void set_suction(bool state) {
        std_msgs::msg::Bool msg;
        msg.data = state;
        suction_pub_->publish(msg);
    }

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<PalletizingServer>(options);
    node->init();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
