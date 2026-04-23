import os
from launch import LaunchDescription
from launch.actions import TimerAction  # 引入 TimerAction 用于延时
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("kuka", package_name="kuka_moveit_config")
        .robot_description(file_path="config/assembly_isaac_sim.urdf")
        .to_moveit_configs()
    )

    palletizing_server_node = Node(
        package="kuka_control",
        executable="palletizing_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    master_node = Node(
        package="palletizing_brain",
        executable="master_node",
        output="screen",
    )

    task_publisher_client_node = Node(
        package="kuka_control",
        executable="task_publisher_client",
        output="screen",
    )

    # 使用 TimerAction 延迟 10 秒启动后面的节点
    delayed_nodes = TimerAction(
        period=10.0,
        actions=[
            master_node,
            task_publisher_client_node
        ]
    )

    return LaunchDescription([
        palletizing_server_node,  # 立即启动 Server
        delayed_nodes,            # 10秒后启动其他节点
    ])