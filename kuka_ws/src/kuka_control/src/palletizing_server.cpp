#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <tuple>
#include <cmath>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
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
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
            move_group_->setEndEffectorLink("ee");

            // 全局默认规划设置
            move_group_->setPlanningTime(15.0);
            move_group_->setNumPlanningAttempts(10);

            if (!move_to_home_position()) {
                RCLCPP_ERROR(this->get_logger(), "严重错误：无法运动到初始位置，系统挂起！");
                return;
            }

            auto current_state = move_group_->getCurrentState(10.0); 
            
            if (current_state != nullptr) {
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
    std::shared_ptr<moveit::core::RobotState> virtual_future_state_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub_;
    rclcpp_action::Server<PalletTask>::SharedPtr action_server_;

    const std::string PLANNING_GROUP = "arm";
    const std::string KINEMATIC_TIP_LINK = "link_6"; // 保留原有约束系
    const int BLEND_RADIUS_POINTS = 5; // 修改裁切点数为 5
    std::atomic<bool> system_ready_{false}; 

    struct ExecutionTask {
        moveit_msgs::msg::RobotTrajectory approach_traj;
        moveit_msgs::msg::RobotTrajectory place_traj;
        std::shared_ptr<GoalHandlePalletTask> goal_handle;
    };
    std::queue<ExecutionTask> task_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    bool stop_execution_ = false;
    std::thread execution_thread_;

    // 延长高度确认的超时时间至 15 秒，避免轨迹执行变慢时误报超时
    void wait_for_z_arrival(double target_z, double tolerance = 0.03, int timeout_ms = 15000) {
        int steps = timeout_ms / 100;
        for (int i = 0; i < steps; ++i) {
            if (!rclcpp::ok() || stop_execution_) break;
            try {
                auto current_pose = move_group_->getCurrentPose("ee").pose;
                if (std::abs(current_pose.position.z - target_z) < tolerance) {
                    RCLCPP_INFO(this->get_logger(), "物理末端已到位: %.3f", current_pose.position.z);
                    return;
                }
            } catch (...) {}
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_WARN(this->get_logger(), "到达高度超时！强行继续。");
    }

    bool move_to_home_position() {
        std::vector<double> home_joints = {
            45.0 * M_PI / 180.0, -90.0 * M_PI / 180.0, 90.0 * M_PI / 180.0,
             0.0 * M_PI / 180.0,  90.0 * M_PI / 180.0,  0.0 * M_PI / 180.0
        };

        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->clearPathConstraints(); 
        move_group_->setJointValueTarget(home_joints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const PalletTask::Goal> goal) {
        if (!system_ready_) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePalletTask> /*goal_handle*/) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 保留提前规划逻辑
    void handle_accepted(const std::shared_ptr<GoalHandlePalletTask> goal_handle) {
        std::thread{ [this, goal_handle]() {
            const auto goal = goal_handle->get_goal();
            
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (task_queue_.empty() && system_ready_) {
                    auto current_state = move_group_->getCurrentState(2.0);
                    if (current_state) *virtual_future_state_ = *current_state;
                }
            }

            // 修改安全高度
            double safe_transit_z = goal->height_limit + 0.6;

            geometry_msgs::msg::Pose pre_approach_pose = goal->start_pose; pre_approach_pose.position.z = safe_transit_z;
            geometry_msgs::msg::Pose pick_pose = goal->start_pose;
            geometry_msgs::msg::Pose lift_pose = goal->start_pose; lift_pose.position.z = safe_transit_z;
            geometry_msgs::msg::Pose hover_place_pose = goal->end_pose; hover_place_pose.position.z = safe_transit_z;
            geometry_msgs::msg::Pose place_pose = goal->end_pose;

            moveit_msgs::msg::RobotTrajectory phase1_msg, phase3_msg;
            
            bool plan_success = plan_and_blend_approach(virtual_future_state_, pre_approach_pose, pick_pose, phase1_msg);
            if (plan_success) {
                plan_success = plan_and_blend_place(virtual_future_state_, lift_pose, hover_place_pose, place_pose, phase3_msg);
            }

            if (plan_success) {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                task_queue_.push({phase1_msg, phase3_msg, goal_handle});
                queue_cv_.notify_one(); 
                RCLCPP_INFO(this->get_logger(), ">>> 任务轨迹规划完毕 (箱子: %s) <<<", goal->box_id.c_str());
            } else {
                auto result = std::make_shared<PalletTask::Result>();
                result->success = false;
                result->message = "轨迹规划由于系统停止而中止";
                goal_handle->abort(result);
            }
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
            }

            const auto goal = current_task.goal_handle->get_goal();
            auto feedback = std::make_shared<PalletTask::Feedback>();

            feedback->phase_description = "接近并抓取: " + goal->box_id;
            current_task.goal_handle->publish_feedback(feedback);
            
            moveit::planning_interface::MoveGroupInterface::Plan plan1;
            plan1.trajectory_ = current_task.approach_traj;
            
            // 执行失败持续重试逻辑（接近阶段）
            while (rclcpp::ok() && !stop_execution_) {
                if (move_group_->execute(plan1) == moveit::core::MoveItErrorCode::SUCCESS) {
                    break;
                }
                RCLCPP_WARN(this->get_logger(), "接近轨迹执行失败，2秒后持续重试...");
                std::this_thread::sleep_for(2000ms);
            }

            wait_for_z_arrival(goal->start_pose.position.z);
            std::this_thread::sleep_for(400ms); 
            
            set_suction(true);
            if (!goal->box_id.empty()) {
                move_group_->attachObject(goal->box_id, "ee");
            }
            std::this_thread::sleep_for(500ms); 

            feedback->phase_description = "转移并放置: " + goal->box_id;
            current_task.goal_handle->publish_feedback(feedback);
            
            moveit::planning_interface::MoveGroupInterface::Plan plan3;
            plan3.trajectory_ = current_task.place_traj;
            
            // 执行失败持续重试逻辑（放置阶段）
            while (rclcpp::ok() && !stop_execution_) {
                if (move_group_->execute(plan3) == moveit::core::MoveItErrorCode::SUCCESS) {
                    break;
                }
                RCLCPP_WARN(this->get_logger(), "放置轨迹执行失败，2秒后持续重试...");
                std::this_thread::sleep_for(2000ms);
            }

            wait_for_z_arrival(goal->end_pose.position.z);
            std::this_thread::sleep_for(400ms);

            set_suction(false);
            if (!goal->box_id.empty()) {
                move_group_->detachObject(goal->box_id);
            }
            std::this_thread::sleep_for(500ms); 

            auto result = std::make_shared<PalletTask::Result>();
            result->success = true;
            result->message = "任务执行成功: " + goal->box_id;
            current_task.goal_handle->succeed(result);
        }
    }

    void set_suction(bool state) {
        std_msgs::msg::Bool msg;
        msg.data = state;
        suction_pub_->publish(msg);
    }

    // ================== 核心算法 ==================
    bool plan_ompl_sim(std::shared_ptr<moveit::core::RobotState>& sim_state, geometry_msgs::msg::Pose target, moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        move_group_->setPlanningTime(15.0); // 增加规划时间
        move_group_->setNumPlanningAttempts(10); // 增加并行计算尝试次数

        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setStartState(*sim_state);
        move_group_->setPoseTarget(target);
        
        geometry_msgs::msg::Quaternion downward_orientation;
        downward_orientation.x = 0.0; downward_orientation.y = 1.0; downward_orientation.z = 0.0; downward_orientation.w = 0.0;

        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = KINEMATIC_TIP_LINK;
        ocm.header.frame_id = move_group_->getPlanningFrame();
        ocm.orientation = downward_orientation;
        ocm.absolute_x_axis_tolerance = 0.1; ocm.absolute_y_axis_tolerance = 0.1; ocm.absolute_z_axis_tolerance = 3.14; ocm.weight = 1.0;
        
        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.orientation_constraints.push_back(ocm);
        move_group_->setPathConstraints(path_constraints);
        
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success && !plan.trajectory_.joint_trajectory.points.empty()) {
            sim_state->setJointGroupPositions(PLANNING_GROUP, plan.trajectory_.joint_trajectory.points.back().positions);
        }
        move_group_->clearPathConstraints();
        return success;
    }

    // 针对层级控制新增的关节空间带约束规划函数
    bool plan_ompl_joint_sim(std::shared_ptr<moveit::core::RobotState>& sim_state, const std::vector<double>& joint_target, moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        move_group_->setPlanningTime(15.0); // 增加规划时间
        move_group_->setNumPlanningAttempts(10); // 增加并行计算尝试次数

        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setStartState(*sim_state);
        move_group_->setJointValueTarget(joint_target);
        
        geometry_msgs::msg::Quaternion downward_orientation;
        downward_orientation.x = 0.0; downward_orientation.y = 1.0; downward_orientation.z = 0.0; downward_orientation.w = 0.0;

        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = KINEMATIC_TIP_LINK;
        ocm.header.frame_id = move_group_->getPlanningFrame();
        ocm.orientation = downward_orientation;
        ocm.absolute_x_axis_tolerance = 0.1; ocm.absolute_y_axis_tolerance = 0.1; ocm.absolute_z_axis_tolerance = 3.14; ocm.weight = 1.0;
        
        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.orientation_constraints.push_back(ocm);
        move_group_->setPathConstraints(path_constraints);
        
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success && !plan.trajectory_.joint_trajectory.points.empty()) {
            sim_state->setJointGroupPositions(PLANNING_GROUP, plan.trajectory_.joint_trajectory.points.back().positions);
        }
        move_group_->clearPathConstraints();
        return success;
    }

    bool plan_pilz_sim(std::shared_ptr<moveit::core::RobotState>& sim_state, geometry_msgs::msg::Pose target, moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        move_group_->setPlanningTime(15.0); 
        move_group_->setNumPlanningAttempts(10); 

        move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_->setPlannerId("LIN");
        move_group_->setStartState(*sim_state);
        move_group_->setPoseTarget(target);
        
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success && !plan.trajectory_.joint_trajectory.points.empty()) {
            sim_state->setJointGroupPositions(PLANNING_GROUP, plan.trajectory_.joint_trajectory.points.back().positions);
        }
        return success;
    }

    bool blend_and_retime_trajectories(
        const std::vector<std::tuple<moveit_msgs::msg::RobotTrajectory, int, int>>& trajs_info,
        moveit::core::RobotState starting_state,
        double v_scale, double a_scale,
        moveit_msgs::msg::RobotTrajectory& out_msg) {
        
        robot_trajectory::RobotTrajectory merged_traj(move_group_->getRobotModel(), PLANNING_GROUP);
        moveit::core::RobotState ref_state = starting_state;

        for (const auto& [msg, trim_start, trim_end] : trajs_info) {
            robot_trajectory::RobotTrajectory temp_traj(move_group_->getRobotModel(), PLANNING_GROUP);
            temp_traj.setRobotTrajectoryMsg(ref_state, msg);
            
            int point_count = temp_traj.getWayPointCount();
            int actual_trim_start = trim_start;
            int actual_trim_end = trim_end;

            if (point_count <= actual_trim_start + actual_trim_end + 1) {
                actual_trim_start = 0; actual_trim_end = 0;
            }

            for (int i = actual_trim_start; i < point_count - actual_trim_end; ++i) {
                if (actual_trim_start == 0 && i == 0 && merged_traj.getWayPointCount() > 0) continue; 
                merged_traj.addSuffixWayPoint(temp_traj.getWayPoint(i), 0.0);
            }
            
            if (merged_traj.getWayPointCount() > 0) {
                ref_state = merged_traj.getLastWayPoint();
            }
        }

        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (totg.computeTimeStamps(merged_traj, v_scale, a_scale)) { 
            merged_traj.getRobotTrajectoryMsg(out_msg);
            return true;
        }
        return false;
    }

    bool plan_and_blend_approach(std::shared_ptr<moveit::core::RobotState>& state, geometry_msgs::msg::Pose pre, geometry_msgs::msg::Pose pick, moveit_msgs::msg::RobotTrajectory& out_traj) {
        // 无限重试，直到规划成功或程序退出
        while (rclcpp::ok() && !stop_execution_) {
            auto working_state = std::make_shared<moveit::core::RobotState>(*state);
            moveit::planning_interface::MoveGroupInterface::Plan plan_approach, plan_descend;
            bool success = true;

            success &= plan_ompl_sim(working_state, pre, plan_approach);
            success &= plan_pilz_sim(working_state, pick, plan_descend);

            if (success) {
                std::vector<std::tuple<moveit_msgs::msg::RobotTrajectory, int, int>> trajs_to_blend = {
                    {plan_approach.trajectory_, 0, BLEND_RADIUS_POINTS},
                    {plan_descend.trajectory_, BLEND_RADIUS_POINTS, 0}
                };

                if (blend_and_retime_trajectories(trajs_to_blend, *state, 0.9, 0.9, out_traj)) {
                    *state = *working_state; 
                    return true;
                }
            }
            RCLCPP_WARN(this->get_logger(), "接近轨迹(Phase1)规划失败，正在持续重试...");
            std::this_thread::sleep_for(1000ms);
        }
        return false; 
    }

    // ================== 层级控制嵌入点 ==================
    bool plan_and_blend_place(std::shared_ptr<moveit::core::RobotState>& state, geometry_msgs::msg::Pose lift, geometry_msgs::msg::Pose hover, geometry_msgs::msg::Pose place, moveit_msgs::msg::RobotTrajectory& out_traj) {
        // 无限重试，直到规划成功或程序退出
        while (rclcpp::ok() && !stop_execution_) {
            auto working_state = std::make_shared<moveit::core::RobotState>(*state);
            
            moveit::planning_interface::MoveGroupInterface::Plan plan_lift, plan_transit_j1, plan_transit_rest, plan_place;
            bool success = true;

            // 1. 提起 (Pilz)
            success &= plan_pilz_sim(working_state, lift, plan_lift);

            // 2. 求解悬停点的目标关节角
            std::vector<double> target_joints;
            if (success) {
                moveit::core::RobotState ik_state(*working_state);
                bool found_ik = ik_state.setFromIK(move_group_->getRobotModel()->getJointModelGroup(PLANNING_GROUP), hover, "ee", 2.0);
                if (found_ik) {
                    ik_state.copyJointGroupPositions(PLANNING_GROUP, target_joints);
                } else {
                    success = false;
                }
            }

            // 3. 层级转移第一段：J1 轴优先到位
            if (success) {
                std::vector<double> current_joints;
                working_state->copyJointGroupPositions(PLANNING_GROUP, current_joints);
                std::vector<double> j1_only_joints = current_joints;
                j1_only_joints[0] = target_joints[0]; // 仅替换底座 J1 的目标角度
                
                success &= plan_ompl_joint_sim(working_state, j1_only_joints, plan_transit_j1);
            }

            // 4. 层级转移第二段：剩余轴伸展
            if (success) {
                success &= plan_ompl_joint_sim(working_state, target_joints, plan_transit_rest);
            }

            // 5. 放置 (Pilz)
            if (success) {
                success &= plan_pilz_sim(working_state, place, plan_place);
            }

            if (success) {
                // 将 4 段轨迹进行融合拼接
                std::vector<std::tuple<moveit_msgs::msg::RobotTrajectory, int, int>> trajs_to_blend = {
                    {plan_lift.trajectory_, 0, BLEND_RADIUS_POINTS},
                    {plan_transit_j1.trajectory_, BLEND_RADIUS_POINTS, BLEND_RADIUS_POINTS},
                    {plan_transit_rest.trajectory_, BLEND_RADIUS_POINTS, BLEND_RADIUS_POINTS},
                    {plan_place.trajectory_, BLEND_RADIUS_POINTS, 0}
                };

                if (blend_and_retime_trajectories(trajs_to_blend, *state, 1.0, 1.0, out_traj)) {
                    *state = *working_state;
                    return true;
                }
            }
            RCLCPP_WARN(this->get_logger(), "放置轨迹(Phase3)规划失败，正在持续重试...");
            std::this_thread::sleep_for(1000ms);
        }
        return false;
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