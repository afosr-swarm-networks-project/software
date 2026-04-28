from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_path("rf_agent_bringup")
    default_config = str(pkg / "config" / "client.yaml")

    config_file = LaunchConfiguration("config_file")
    iq_topic    = LaunchConfiguration("iq_topic")
    scores_topic = LaunchConfiguration("scores_topic")

    pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg / "launch" / "pipeline.launch.py")),
        launch_arguments={
            "config_file":  config_file,
            "iq_topic":     iq_topic,
            "scores_topic": scores_topic,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the node parameters YAML file",
        ),
        DeclareLaunchArgument("iq_topic",     default_value="iq"),
        DeclareLaunchArgument("scores_topic", default_value="scores"),

        Node(
            package="uhd_usrp",
            executable="usrp_driver_node",
            name="usrp_driver_node",
            output="screen",
            parameters=[config_file],
            remappings=[("iq", iq_topic)],
        ),

        pipeline_launch,
    ])
