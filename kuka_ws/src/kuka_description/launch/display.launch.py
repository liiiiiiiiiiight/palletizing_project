from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    model_arg = DeclareLaunchArgument('model', default_value='')

    # 获取 URDF 文件路径
    robot_description_content = Command(
        ['cat ', PathJoinSubstitution([FindPackageShare('kuka_description'), 'urdf', 'kuka_description.urdf'])]
    )

    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 关节状态发布 GUI 节点
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2 节点
    rviz_config = PathJoinSubstitution([FindPackageShare('kuka_description'), 'urdf.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])