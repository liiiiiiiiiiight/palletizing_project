import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 自动加载 kuka_moveit_config 包下的所有配置 (URDF, SRDF, kinematics.yaml 等)
    moveit_config = (
        MoveItConfigsBuilder("kuka", package_name="kuka_moveit_config")
        .robot_description(file_path="config/assembly_isaac_sim.urdf") # 指向urdf
        .to_moveit_configs()
    )

    # 启动你的服务端节点，并将参数字典传递给它
    palletizing_server_node = Node(
        package="kuka_control",
        executable="palletizing_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True} # 因为你用的是 Isaac Sim 仿真，建议开启使用仿真时间
        ],
    )

    return LaunchDescription([palletizing_server_node])