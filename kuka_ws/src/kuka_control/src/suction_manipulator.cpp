#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("suction_manipulator_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    const std::string PLANNING_GROUP = "arm";      
    const std::string SUCTION_LINK = "ee";         
    const std::string KINEMATIC_TIP_LINK = "link_6"; 
    const std::string TARGET_OBJECT = "grasp_box_1";         

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    move_group.setEndEffectorLink(SUCTION_LINK);  

    auto suction_pub = node->create_publisher<std_msgs::msg::Bool>("/suction_cmd", 10);
    auto set_suction = [&suction_pub, &node](bool state) {
        std_msgs::msg::Bool msg;
        msg.data = state;
        suction_pub->publish(msg); 
        RCLCPP_INFO(node->get_logger(), ">>> [动作] 吸盘状态: %s <<<", state ? "ON" : "OFF");
        std::this_thread::sleep_for(500ms); 
    };

    geometry_msgs::msg::Quaternion downward_orientation;
    downward_orientation.x = 1.0; downward_orientation.y = 0.0; downward_orientation.z = 0.0; downward_orientation.w = 0.0;

    const double SAFE_TRANSIT_Z = 2.4; 

    // 关键位姿定义
    geometry_msgs::msg::Pose pre_approach_pose, pick_pose, lift_pose, hover_place_pose, place_pose, retreat_pose;
    
    pre_approach_pose.orientation = downward_orientation;
    pre_approach_pose.position.x = 1.7; pre_approach_pose.position.y = -0.4; pre_approach_pose.position.z = SAFE_TRANSIT_Z; 
    
    pick_pose = pre_approach_pose; pick_pose.position.z = 0.615; 
    lift_pose = pick_pose; lift_pose.position.z = SAFE_TRANSIT_Z; 
    
    hover_place_pose.orientation = downward_orientation;
    hover_place_pose.position.x = 0.27; hover_place_pose.position.y = 2.1; hover_place_pose.position.z = SAFE_TRANSIT_Z; 
    
    place_pose = hover_place_pose; place_pose.position.z = 0.615; 
    retreat_pose = hover_place_pose;


    // ================= 工具集 A：鲁棒物理执行 (针对离散动作/撤离) =================
    auto robust_execute_pilz = [&](geometry_msgs::msg::Pose target_pose, const std::string& step_name) {
        move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group.setPlannerId("LIN");
        move_group.setMaxVelocityScalingFactor(1.0); // 撤离阶段速度拉满
        move_group.setMaxAccelerationScalingFactor(1.0);
        
        while (rclcpp::ok()) {
            move_group.setStartStateToCurrentState(); 
            move_group.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                if (move_group.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(node->get_logger(), "[%s] 执行成功！", step_name.c_str());
                    std::this_thread::sleep_for(200ms); 
                    break;
                }
            }
            RCLCPP_WARN(node->get_logger(), "[%s] 执行或规划失败，正在重试...", step_name.c_str());
            std::this_thread::sleep_for(1s);
        }
    };

    // ================= 工具集 B：虚拟状态推演 (产生规划轨迹) =================
    auto plan_ompl_sim = [&](moveit::core::RobotStatePtr& sim_state, geometry_msgs::msg::Pose target, moveit::planning_interface::MoveGroupInterface::Plan& plan) -> bool {
        move_group.setPlanningPipelineId("ompl");
        move_group.setPlannerId("RRTConnectkConfigDefault");
        move_group.setStartState(*sim_state);
        move_group.setPoseTarget(target);
        
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = KINEMATIC_TIP_LINK;
        ocm.header.frame_id = move_group.getPlanningFrame();
        ocm.orientation = downward_orientation;
        ocm.absolute_x_axis_tolerance = 0.1; ocm.absolute_y_axis_tolerance = 0.1; ocm.absolute_z_axis_tolerance = 3.14; ocm.weight = 1.0;
        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.orientation_constraints.push_back(ocm);
        move_group.setPathConstraints(path_constraints);
        
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            sim_state->setJointGroupPositions(PLANNING_GROUP, plan.trajectory_.joint_trajectory.points.back().positions);
        }
        move_group.clearPathConstraints();
        return success;
    };

    auto plan_pilz_sim = [&](moveit::core::RobotStatePtr& sim_state, geometry_msgs::msg::Pose target, moveit::planning_interface::MoveGroupInterface::Plan& plan) -> bool {
        move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group.setPlannerId("LIN");
        move_group.setStartState(*sim_state);
        move_group.setPoseTarget(target);
        
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            sim_state->setJointGroupPositions(PLANNING_GROUP, plan.trajectory_.joint_trajectory.points.back().positions);
        }
        return success;
    };

    // ================= 工具集 C：轨迹裁切与平滑重排 (复用模块) =================
    // 传入的元组结构: {轨迹数据, 头部裁切点数, 尾部裁切点数}
    auto blend_and_retime_trajectories = [&](
        const std::vector<std::tuple<moveit_msgs::msg::RobotTrajectory, int, int>>& trajs_info,
        moveit::core::RobotState starting_state,
        double v_scale, double a_scale,
        moveit_msgs::msg::RobotTrajectory& out_msg) -> bool {
        
        robot_trajectory::RobotTrajectory merged_traj(move_group.getRobotModel(), PLANNING_GROUP);
        moveit::core::RobotState ref_state = starting_state;

        for (const auto& [msg, trim_start, trim_end] : trajs_info) {
            robot_trajectory::RobotTrajectory temp_traj(move_group.getRobotModel(), PLANNING_GROUP);
            temp_traj.setRobotTrajectoryMsg(ref_state, msg);
            
            int point_count = temp_traj.getWayPointCount();
            int actual_trim_start = trim_start;
            int actual_trim_end = trim_end;

            // 轨迹过短保护
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
    };

    // ==========================================================
    // ================= 实机任务流程开始 ========================
    // ==========================================================
    RCLCPP_INFO(node->get_logger(), "=== 开始执行抓取放置任务 ===");
    const int BLEND_RADIUS_POINTS = 15; // 过渡圆角半径参数

    // ---------------- 阶段 1: 融合规划 [接近 -> 下降] ----------------
    RCLCPP_INFO(node->get_logger(), "=== 计算阶段1：无停顿接近轨迹 ===");
    moveit::planning_interface::MoveGroupInterface::Plan plan_approach, plan_descend;
    moveit_msgs::msg::RobotTrajectory fused_phase1_msg;
    bool phase1_planned = false;

    while (rclcpp::ok() && !phase1_planned) {
        moveit::core::RobotStatePtr sim_state = move_group.getCurrentState();
        bool success = true;

        success &= plan_ompl_sim(sim_state, pre_approach_pose, plan_approach);
        success &= plan_pilz_sim(sim_state, pick_pose, plan_descend);

        if (success) {
            // 将 OMPL 的尾部切角，Pilz LIN 的头部切角，合成平滑下降
            std::vector<std::tuple<moveit_msgs::msg::RobotTrajectory, int, int>> trajs_to_blend = {
                {plan_approach.trajectory_, 0, BLEND_RADIUS_POINTS},
                {plan_descend.trajectory_, BLEND_RADIUS_POINTS, 0}
            };

            // 注意：下降接触物体如果 100% 速度可能会有轻微冲击，这里设为 0.9 兼顾极速与安全
            if (blend_and_retime_trajectories(trajs_to_blend, *move_group.getCurrentState(), 1, 1, fused_phase1_msg)) {
                phase1_planned = true;
                RCLCPP_INFO(node->get_logger(), ">>> 接近轨迹融合完成！ <<<");
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "阶段1规划失败，重试中...");
            std::this_thread::sleep_for(500ms);
        }
    }

    // 执行阶段 1
    moveit::planning_interface::MoveGroupInterface::Plan phase1_plan;
    phase1_plan.trajectory_ = fused_phase1_msg;
    if (move_group.execute(phase1_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
         RCLCPP_ERROR(node->get_logger(), "阶段1执行失败！");
    }

    // ---------------- 阶段 2: 抓取 ----------------
    set_suction(true);
    std::vector<std::string> touch_links = move_group.getRobotModel()->getLinkModelNames();
    std::this_thread::sleep_for(500ms);
    move_group.attachObject(TARGET_OBJECT, SUCTION_LINK, touch_links);
    
    // ---------------- 阶段 3: 融合规划 [提起 -> 转移 -> 放置] ----------------
    RCLCPP_INFO(node->get_logger(), "=== 计算阶段3：无停顿放置轨迹 ===");
    moveit::planning_interface::MoveGroupInterface::Plan plan_lift, plan_transit, plan_place;
    moveit_msgs::msg::RobotTrajectory fused_phase3_msg;
    bool phase3_planned = false;

    while (rclcpp::ok() && !phase3_planned) {
        moveit::core::RobotStatePtr sim_state = move_group.getCurrentState();
        bool success = true;

        success &= plan_pilz_sim(sim_state, lift_pose, plan_lift);
        success &= plan_ompl_sim(sim_state, hover_place_pose, plan_transit);
        success &= plan_pilz_sim(sim_state, place_pose, plan_place);

        if (success) {
            std::vector<std::tuple<moveit_msgs::msg::RobotTrajectory, int, int>> trajs_to_blend = {
                {plan_lift.trajectory_, 0, BLEND_RADIUS_POINTS},
                {plan_transit.trajectory_, BLEND_RADIUS_POINTS, BLEND_RADIUS_POINTS},
                {plan_place.trajectory_, BLEND_RADIUS_POINTS, 0}
            };

            // 此处无障碍纯空中转移，速度 100% 拉满
            if (blend_and_retime_trajectories(trajs_to_blend, *move_group.getCurrentState(), 1.0, 1.0, fused_phase3_msg)) {
                phase3_planned = true;
                RCLCPP_INFO(node->get_logger(), ">>> 放置轨迹融合完成！ <<<");
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "阶段3规划失败，重试中...");
            std::this_thread::sleep_for(500ms);
        }
    }

    // 执行阶段 3
    moveit::planning_interface::MoveGroupInterface::Plan phase3_plan;
    phase3_plan.trajectory_ = fused_phase3_msg;
    if (move_group.execute(phase3_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
         RCLCPP_ERROR(node->get_logger(), "阶段3执行失败！");
    }

    // ---------------- 阶段 4: 释放与撤离 ----------------
    set_suction(false);
    move_group.detachObject(TARGET_OBJECT);
    
    robust_execute_pilz(retreat_pose, "撤离 (Pilz LIN )");

    RCLCPP_INFO(node->get_logger(), "=== 任务完成 ===");
    rclcpp::shutdown();
    spinner.join(); 
    return 0;
}