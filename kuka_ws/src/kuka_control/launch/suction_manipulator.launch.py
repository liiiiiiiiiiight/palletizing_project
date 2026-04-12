from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 显式指定 MoveIt 配置包名：kuka_moveit_config
    moveit_config = (
        MoveItConfigsBuilder(robot_name="assembly_moveit", package_name="kuka_moveit_config")
        .to_moveit_configs()
    )

    suction_node = Node(
        package="kuka_control",
        executable="suction_manipulator",
        name="suction_manipulator_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,  # 注入关节加速度和速度极限
        ],
    )

    return LaunchDescription([suction_node])