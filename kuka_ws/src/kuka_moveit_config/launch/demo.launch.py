import launch
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    use_isaac_sim = launch.substitutions.LaunchConfiguration("use_isaac_sim")

    moveit_config = (
        MoveItConfigsBuilder("assembly_moveit", package_name="kuka_moveit_config")
        .robot_description(mappings={"use_isaac_sim": use_isaac_sim})
        .to_moveit_configs()
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "use_isaac_sim",
                default_value="false",
                description="Use Isaac Sim hardware interface instead of fake controller",
            ),
        ]
        + generate_demo_launch(moveit_config).entities
    )
