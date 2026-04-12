#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "kuka_interfaces/action/pallet_task.hpp"
#include <string>
#include <unordered_map>

using PalletTask = kuka_interfaces::action::PalletTask;
using GoalHandlePalletTask = rclcpp_action::ClientGoalHandle<PalletTask>;

class TaskManagerClient : public rclcpp::Node
{
public:
    TaskManagerClient() : Node("task_manager_client") {
        client_ptr_ = rclcpp_action::create_client<PalletTask>(this, "pallet_task");
        
        // 启动一个非阻塞的轮询定时器，检查 Action Server 是否上线
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TaskManagerClient::check_server_and_start, this));
    }

private:
    rclcpp_action::Client<PalletTask>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::TimerBase::SharedPtr dispatch_timer_;
    
    int current_i_ = 0;
    int current_j_ = 0;

    // 持久化管理 Action 句柄，防止被 ROS2 垃圾回收导致丢失 Result 回调
    std::unordered_map<std::string, GoalHandlePalletTask::SharedPtr> active_goals_;

    void check_server_and_start() {
        if (!client_ptr_->action_server_is_ready()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "等待 Action Server 上线...");
            return;
        }

        // Server 已就绪，取消检查定时器
        check_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Action Server 已连接！开始【每 3 秒】强制下发一个任务...");

        // 启动 3 秒循环下发定时器
        dispatch_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&TaskManagerClient::timer_callback, this));
        
        // 立即手动触发第一个任务，不用死等第一个 3 秒
        timer_callback();
    }

    void timer_callback() {
        // 检查是否所有任务都已完成 (3x5 = 15个任务)
        if (current_i_ >= 3) {
            RCLCPP_INFO(this->get_logger(), "== 所有 15 个箱子的码垛任务已全部推入服务端队列，停止下发！ ==");
            dispatch_timer_->cancel(); // 任务发完，关闭定时器
            return;
        }

        // --- 基于 box_creator 中的参数生成当前箱子的起止点 ---
        double box_x = 0.3, box_y = 0.2, box_z = 0.2; 
        double max_x = 1.75;
        double max_y = -0.4;
        double base_z = 0.507;
        double gap = 0.005; 

        double sx = max_x - current_i_ * (box_x + gap);
        double sy = max_y - current_j_ * (box_y + gap);
        double sz = base_z + box_z/2 + 0.005;

        double ex = sx - 1.48;
        double ey = sy + 2.5;
        double ez = sz;

        std::string box_id = "grasp_box_" + std::to_string(current_i_) + "_" + std::to_string(current_j_);
        
        // 执行下发
        send_single_goal(box_id, sx, sy, sz, ex, ey, ez, 10.0, 1.8);

        // 索引自增，为 3 秒后的下一次下发做准备
        current_j_++;
        if (current_j_ >= 5) {
            current_j_ = 0;
            current_i_++;
        }
    }

    void send_single_goal(const std::string& box_id, double sx, double sy, double sz, 
                          double ex, double ey, double ez, double mass, double h_limit) {
        auto goal_msg = PalletTask::Goal();
        
        goal_msg.box_id = box_id;
        goal_msg.start_pose.position.x = sx; goal_msg.start_pose.position.y = sy; goal_msg.start_pose.position.z = sz;
        goal_msg.start_pose.orientation.y = 1.0; 
        goal_msg.start_pose.orientation.x = 0; goal_msg.start_pose.orientation.z = 0; goal_msg.start_pose.orientation.w = 0;

        goal_msg.end_pose.position.x = ex; goal_msg.end_pose.position.y = ey; goal_msg.end_pose.position.z = ez;
        goal_msg.end_pose.orientation.y = 1.0;
        goal_msg.end_pose.orientation.x = 0; goal_msg.end_pose.orientation.z = 0; goal_msg.end_pose.orientation.w = 0;

        goal_msg.box_size.x = 0.3; goal_msg.box_size.y = 0.2; goal_msg.box_size.z = 0.2;
        goal_msg.box_mass = mass; goal_msg.height_limit = h_limit;

        auto send_goal_options = rclcpp_action::Client<PalletTask>::SendGoalOptions();

        // 1. Goal Response 回调：任务被接收时，将句柄存入 map 中续命
        send_goal_options.goal_response_callback =
            [this, box_id](const GoalHandlePalletTask::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "任务被服务端拒绝: %s", box_id.c_str());
                } else {
                    active_goals_[box_id] = goal_handle;
                    RCLCPP_INFO(this->get_logger(), "[%s] 已成功推入服务端排队", box_id.c_str());
                }
            };

        // 2. Feedback 回调：仅做日志打印，不再用于逻辑触发
        send_goal_options.feedback_callback =
            [this, box_id](GoalHandlePalletTask::SharedPtr, const std::shared_ptr<const PalletTask::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "[%s] 状态更新: %s", box_id.c_str(), feedback->phase_description.c_str());
            };

        // 3. Result 回调：任务彻底执行完毕，释放句柄内存
        send_goal_options.result_callback =
            [this, box_id](const GoalHandlePalletTask::WrappedResult & result) {
                
                // 从活跃列表中移除该任务
                active_goals_.erase(box_id);

                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), ">>> 物理任务彻底完结: %s <<<", box_id.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "!!! 任务执行失败或被中断: %s !!!", box_id.c_str());
                }
            };

        // 异步发送目标
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskManagerClient>());
    rclcpp::shutdown();
    return 0;
}