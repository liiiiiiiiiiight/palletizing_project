#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>  // 新增：用于节点关闭时的微小延时

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

class BoxCreator : public rclcpp::Node
{
public:
    BoxCreator() : Node("box_creator")
    {
        scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", 10);

        RCLCPP_INFO(this->get_logger(), "BoxCreator node started. Waiting 2 seconds for MoveIt to connect...");

        // 使用单次定时器延迟发布
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                addBoxes();
                timer_->cancel();
            });

        // 注册关闭时的回调函数 (拦截 Ctrl+C 或节点结束)
        rclcpp::on_shutdown([this]() {
            RCLCPP_INFO(this->get_logger(), "Node shutting down. Removing all published boxes...");
            this->removeAllBoxes();
            
            // 重要：给 ROS 2 底层 DDS 一点时间发送清除消息，避免节点秒退导致 MoveIt 漏接消息
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        });
    }

private:
    std::vector<std::string> published_box_names_; // 新增：用于记录已发布的所有箱子的 ID

    void addBoxes()
    {
        auto scene_msg = moveit_msgs::msg::PlanningScene();
        scene_msg.is_diff = true; // 差异更新

        // --- 1. 创建 3x5 的箱子阵列 ---
        std::vector<double> box_size = {0.3, 0.2, 0.2}; 

        int num_x = 3; 
        int num_y = 5; 
        
        // 顶点坐标
        double max_x = 1.75;
        double max_y = -0.4;
        double base_z = 0.507;
        double gap = 0.005; 

        // 嵌套循环生成阵列
        for (int i = 0; i < num_x; ++i) {
            for (int j = 0; j < num_y; ++j) {
                double current_x = max_x - i * (box_size[0] + gap);
                double current_y = max_y - j * (box_size[1] + gap);
                
                std::string box_name = "grasp_box_" + std::to_string(i) + "_" + std::to_string(j);
                published_box_names_.push_back(box_name); // 记录名字以便后续清除

                auto box = createBox(box_name, 
                                     box_size, 
                                     {current_x, current_y, base_z});

                scene_msg.world.collision_objects.push_back(box);
            }
        }

        // --- 2. 创建原有的障碍物箱子 ---
        std::string obs_name = "obstacle_box";
        published_box_names_.push_back(obs_name); // 记录障碍物名字

        auto obstacle_box = createBox(obs_name, 
                                      {0.9, 1.1, 1.4},      
                                      {1.45, 0.8, 1.11});   

        scene_msg.world.collision_objects.push_back(obstacle_box);

        // 发布场景
        scene_publisher_->publish(scene_msg);
        RCLCPP_INFO(this->get_logger(), "Successfully published %zu boxes to the planning scene.", 
                    scene_msg.world.collision_objects.size());
    }

    // 新增：移除所有已添加的箱子
    void removeAllBoxes()
    {
        if (published_box_names_.empty()) {
            return;
        }

        auto scene_msg = moveit_msgs::msg::PlanningScene();
        scene_msg.is_diff = true;

        for (const auto& name : published_box_names_) {
            moveit_msgs::msg::CollisionObject collision_obj;
            collision_obj.header.frame_id = "world"; // 需与添加时保持一致
            collision_obj.header.stamp = this->now();
            collision_obj.id = name;
            
            // 核心：设置操作为 REMOVE
            collision_obj.operation = moveit_msgs::msg::CollisionObject::REMOVE; 
            scene_msg.world.collision_objects.push_back(collision_obj);
        }

        scene_publisher_->publish(scene_msg);
        RCLCPP_INFO(this->get_logger(), "Successfully sent REMOVE commands for %zu boxes.", 
                    published_box_names_.size());
        
        // 清空列表
        published_box_names_.clear();
    }

    // 辅助函数：根据参数生成 CollisionObject
    moveit_msgs::msg::CollisionObject createBox(const std::string& name, 
                                                const std::vector<double>& size, 
                                                const std::vector<double>& position)
    {
        moveit_msgs::msg::CollisionObject collision_obj;
        collision_obj.header.frame_id = "world";  
        collision_obj.header.stamp = this->now();
        collision_obj.id = name;
        collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = size[0];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = size[1];
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = size[2];

        geometry_msgs::msg::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        pose.orientation.w = 1.0;  

        collision_obj.primitives.push_back(primitive);
        collision_obj.primitive_poses.push_back(pose);

        return collision_obj;
    }

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BoxCreator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}