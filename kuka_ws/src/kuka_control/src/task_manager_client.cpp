#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "kuka_interfaces/action/pallet_task.hpp"

using PalletTask = kuka_interfaces::action::PalletTask;
using GoalHandlePalletTask = rclcpp_action::ClientGoalHandle<PalletTask>;

class TaskManagerClient : public rclcpp::Node
{
public:
    TaskManagerClient() : Node("task_manager_client") {
        client_ptr_ = rclcpp_action::create_client<PalletTask>(this, "pallet_task");
        
        // 启动一个单次定时器，延迟500ms后触发第一个任务
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TaskManagerClient::start_task_sequence, this));
    }

private:
    rclcpp_action::Client<PalletTask>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 用于替代循环的状态变量
    int current_i_ = 0;
    int current_j_ = 0;

    void start_task_sequence() {
        // 只需触发一次，触发后取消定时器
        timer_->cancel();

        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server 未上线，放弃下发。");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Action server 已连接，开始顺序下发任务...");
        send_next_task();
    }

    void send_next_task() {
        // 检查是否所有任务都已完成 (3x5 = 15个任务)
        if (current_i_ >= 3) {
            RCLCPP_INFO(this->get_logger(), "15 个箱子的码垛任务已全部顺序执行完毕！");
            return;
        }

        // --- 基于 box_creator 中的参数生成当前箱子的起止点 ---
        double box_x = 0.3, box_y = 0.2, box_z = 0.2; 
        double max_x = 1.75;
        double max_y = -0.4;
        double base_z = 0.507;
        double gap = 0.005; 

        // 计算起点
        double sx = max_x - current_i_ * (box_x + gap);
        double sy = max_y - current_j_ * (box_y + gap);
        double sz = base_z + box_z/2 + 0.005;

        // 计算终点：向 x 负方向移动 1.48m，y 正方向移动 2.5m，z 不变
        double ex = sx - 1.48;
        double ey = sy + 2.5;
        double ez = sz;

        // 拼装箱子 ID (必须与 box_creator 保持一致)
        std::string box_id = "grasp_box_" + std::to_string(current_i_) + "_" + std::to_string(current_j_);
        
        // 下发当前任务 (质量 10kg, limit 1.8m)
        send_single_goal(box_id, sx, sy, sz, ex, ey, ez, 10.0, 1.8);
    }

    void send_single_goal(const std::string& box_id, double sx, double sy, double sz, 
                          double ex, double ey, double ez, double mass, double h_limit) {
        auto goal_msg = PalletTask::Goal();
        
        goal_msg.box_id = box_id;
        
        goal_msg.start_pose.position.x = sx;
        goal_msg.start_pose.position.y = sy;
        goal_msg.start_pose.position.z = sz;
        goal_msg.start_pose.orientation.y = 1.0; // 默认朝下
        goal_msg.start_pose.orientation.x = 0;
        goal_msg.start_pose.orientation.z = 0;
        goal_msg.start_pose.orientation.w = 0;

        goal_msg.end_pose.position.x = ex;
        goal_msg.end_pose.position.y = ey;
        goal_msg.end_pose.position.z = ez;
        goal_msg.end_pose.orientation.y = 1.0;
        goal_msg.end_pose.orientation.x = 0;
        goal_msg.end_pose.orientation.z = 0;
        goal_msg.end_pose.orientation.w = 0;

        // 箱子尺寸
        goal_msg.box_size.x = 0.3;
        goal_msg.box_size.y = 0.2;
        goal_msg.box_size.z = 0.2;

        goal_msg.box_mass = mass;
        goal_msg.height_limit = h_limit;

        auto send_goal_options = rclcpp_action::Client<PalletTask>::SendGoalOptions();
        
        // 在结果回调中控制后续逻辑
        send_goal_options.result_callback =
            [this, box_id](const GoalHandlePalletTask::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "任务成功完结: %s", box_id.c_str());
                    
                    // 成功后，更新索引以准备下一个任务
                    current_j_++;
                    if (current_j_ >= 5) {
                        current_j_ = 0;
                        current_i_++;
                    }
                    
                    // 触发下一个任务
                    send_next_task();
                } else {
                    // 如果任务失败，停止下发后续任务并报错
                    RCLCPP_ERROR(this->get_logger(), "任务执行失败: %s，中止后续任务下发。", box_id.c_str());
                }
            };

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "已异步下发新任务: %s", box_id.c_str());
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskManagerClient>());
    rclcpp::shutdown();
    return 0;
}