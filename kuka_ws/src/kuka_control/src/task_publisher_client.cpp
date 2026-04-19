#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "palletizing_interfaces/srv/get_task.hpp"
#include "kuka_interfaces/action/pallet_task.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using PalletTask = kuka_interfaces::action::PalletTask;
using GetTask = palletizing_interfaces::srv::GetTask;

class TaskPublisherClient : public rclcpp::Node {
public:
    TaskPublisherClient() : Node("task_publisher_client") {
        action_client_ = rclcpp_action::create_client<PalletTask>(this, "pallet_task");
        service_client_ = this->create_client<GetTask>("get_pallet_task");
        workflow_thread_ = std::thread(&TaskPublisherClient::run_workflow, this);
    }

    ~TaskPublisherClient() {
        if (workflow_thread_.joinable()) workflow_thread_.join();
    }

private:
    rclcpp_action::Client<PalletTask>::SharedPtr action_client_;
    rclcpp::Client<GetTask>::SharedPtr service_client_;
    std::thread workflow_thread_;

    void run_workflow() {
        RCLCPP_INFO(this->get_logger(), "等待 Python 大脑 (Service) 上线...");
        while (!service_client_->wait_for_service(1s)) {
           if (!rclcpp::ok()) return;
        }

        RCLCPP_INFO(this->get_logger(), "等待 C++ 底层控制 (Action Server) 上线...");
        while (!action_client_->wait_for_action_server(1s)) {
            if (!rclcpp::ok()) return;
        }

        // 系统初始化：双工位同时预上料，请求 box_index=0 只生成环境
        RCLCPP_INFO(this->get_logger(), "系统初始化：双工位同时预上料！");
        auto init_req1 = std::make_shared<GetTask::Request>(); init_req1->order_id = 1; init_req1->box_index = 0;
        auto init_req2 = std::make_shared<GetTask::Request>(); init_req2->order_id = 2; init_req2->box_index = 0;
        send_service_request(init_req1);
        send_service_request(init_req2);
        std::this_thread::sleep_for(3s);

        RCLCPP_INFO(this->get_logger(), "所有节点已连接！开始执行订单流水线...");

        for (int order_id = 1; order_id <= 11; ++order_id) {
            RCLCPP_INFO(this->get_logger(), "=======================================");
            RCLCPP_INFO(this->get_logger(), " 准备执行订单 %d", order_id);

            auto req = std::make_shared<GetTask::Request>();
            req->order_id = order_id;
            req->box_index = 1;
            auto result = send_service_request(req);

            if (!result || !result->success) {
                RCLCPP_WARN(this->get_logger(), "获取订单 %d 失败，可能已无数据，跳过...", order_id);
                continue;
            }

            int target_qty = result->target_qty;
            RCLCPP_INFO(this->get_logger(), "订单 %d 开始搬运，共 %d 个箱子", order_id, target_qty);

            for (int box_idx = 1; box_idx <= target_qty; ++box_idx) {
                RCLCPP_INFO(this->get_logger(), "正在呼叫机械臂搬运: 订单 %d, 第 %d / %d 个箱子", order_id, box_idx, target_qty);
                if (box_idx > 1) {
                    req->box_index = box_idx;
                    result = send_service_request(req);
                    if (!result || !result->success) {
                        RCLCPP_ERROR(this->get_logger(), "Python 报错：无法获取箱子 %d 坐标！", box_idx);
                        break;
                    }
                }

                bool success = send_action_command(result, box_idx);
                if (!success) {
                    RCLCPP_ERROR(this->get_logger(), "机械臂执行失败或连接超时，紧急停机！");
                    return;
                }
            }

            RCLCPP_INFO(this->get_logger(), "订单 %d 搬运完成！机械臂立刻转向下一工位...", order_id);
            int next_order_id = order_id + 2;
            if (next_order_id <= 11) {
                std::thread([this, next_order_id]() {
                    RCLCPP_INFO(this->get_logger(), "   [换料后台] 工位清空，倒计时 5 秒准备上新料...");
                    std::this_thread::sleep_for(5s);

                    RCLCPP_INFO(this->get_logger(), "   [换料后台] 5秒已到！呼叫 Python 刷新订单 %d", next_order_id);
                    auto async_req = std::make_shared<GetTask::Request>();
                    async_req->order_id = next_order_id;
                    async_req->box_index = 0;
                    send_service_request(async_req);
                }).detach();
            }
        }

        RCLCPP_INFO(this->get_logger(), "所有 11 个订单已全部执行完毕！");
    }

    std::shared_ptr<GetTask::Response> send_service_request(std::shared_ptr<GetTask::Request> req) {
        auto future = service_client_->async_send_request(req);
        if (future.wait_for(10s) == std::future_status::ready) return future.get();
        return nullptr;
    }

    bool send_action_command(std::shared_ptr<GetTask::Response> task_data, int box_id) {
        if (!task_data) return false;
        auto goal_msg = PalletTask::Goal();

        goal_msg.start_pose.position.x = task_data->pick_x;
        goal_msg.start_pose.position.y = task_data->pick_y;
        goal_msg.start_pose.position.z = task_data->pick_z;
        goal_msg.start_pose.orientation.w = task_data->pick_qw;
        goal_msg.start_pose.orientation.x = task_data->pick_qx;
        goal_msg.start_pose.orientation.y = task_data->pick_qy;
        goal_msg.start_pose.orientation.z = task_data->pick_qz;

        goal_msg.end_pose.position.x = task_data->place_x;
        goal_msg.end_pose.position.y = task_data->place_y;
        goal_msg.end_pose.position.z = task_data->place_z;
        goal_msg.end_pose.orientation.w = task_data->place_qw;
        goal_msg.end_pose.orientation.x = task_data->place_qx;
        goal_msg.end_pose.orientation.y = task_data->place_qy;
        goal_msg.end_pose.orientation.z = task_data->place_qz;

        RCLCPP_INFO(this->get_logger(),
            "[Goal] box_%d | Pick(%.4f, %.4f, %.4f) -> Place(%.4f, %.4f, %.4f) | size(%.3f, %.3f, %.3f)",
            box_id,
            goal_msg.start_pose.position.x, goal_msg.start_pose.position.y, goal_msg.start_pose.position.z,
            goal_msg.end_pose.position.x, goal_msg.end_pose.position.y, goal_msg.end_pose.position.z,
            task_data->box_len, task_data->box_wid, task_data->box_hei);

        goal_msg.box_size.x = task_data->box_len;
        goal_msg.box_size.y = task_data->box_wid;
        goal_msg.box_size.z = task_data->box_hei;
        goal_msg.box_id = "box_" + std::to_string(box_id);
        goal_msg.box_mass = 2.0;
        goal_msg.height_limit = 1.8;

        auto send_goal_options = rclcpp_action::Client<PalletTask>::SendGoalOptions();
        auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

        if (goal_handle_future.wait_for(10s) != std::future_status::ready) return false;

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) return false;

        auto result_future = action_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::hours(1)) != std::future_status::ready) return false;
        auto result = result_future.get();
        return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPublisherClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
