import os
from launch import LaunchDescription
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

    return LaunchDescription([
        palletizing_server_node,
        master_node,
        task_publisher_client_node,
    ])
