import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from pathlib import Path


def _as_float_string(value: str, default: str) -> str:
    text = value.strip() if value is not None else ""
    if not text:
        return default
    return str(float(text))


def _world_config(world_path: str) -> tuple[list[dict[str, str]], str]:
    tree = ET.parse(world_path)
    root = tree.getroot()
    plugin = root.find(".//plugin[@name='rf_gz::RfWorldPlugin']")
    if plugin is None:
      raise RuntimeError(f"rf_gz world plugin not found in {world_path}")

    transmitters = plugin.findall("transmitter")
    default_center_freq_hz = "2400000000.0"
    if transmitters:
        cf_text = transmitters[0].findtext("cf_hz")
        if cf_text:
            default_center_freq_hz = _as_float_string(cf_text, default_center_freq_hz)

    receivers = []
    for receiver in plugin.findall("receiver"):
        name = receiver.attrib.get("name")
        if not name:
            continue
        receivers.append({
            "name": name,
            "fs_hz": _as_float_string(receiver.findtext("fs_hz"), "1000.0"),
        })

    return receivers, default_center_freq_hz


def _launch_setup(context, *args, **kwargs):
    ros_gz_sim_share = get_package_share_path("ros_gz_sim")
    rf_agent_bringup_share = get_package_share_path("rf_agent_bringup")
    world_path = Path(LaunchConfiguration("world").perform(context))
    receivers, _ = _world_config(world_path)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ros_gz_sim_share / "launch" / "gz_sim.launch.py"
        ),
        launch_arguments={
            "gz_args": f"-r {world_path}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    pipeline_launch = rf_agent_bringup_share / "launch" / "pipeline.launch.py"

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    actions = [gz_sim, clock_bridge]
    pipeline_groups = []
    for receiver in receivers:
        receiver_name = receiver["name"]
        iq_topic = f"/rf/{receiver_name}/iq"
        pipeline_groups.append(
            GroupAction([
                PushRosNamespace(receiver_name),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(pipeline_launch),
                    launch_arguments={
                        "iq_topic": iq_topic,
                        "stft_win_s": LaunchConfiguration("stft_win_s").perform(context),
                        "fft_size": LaunchConfiguration("fft_size").perform(context),
                        "hop_size": LaunchConfiguration("hop_size").perform(context),
                        "model_path": LaunchConfiguration("model_path").perform(context),
                        "conf_thresh": LaunchConfiguration("conf_thresh").perform(context),
                        "use_sim_time": "true",
                    }.items(),
                ),
            ])
        )

    actions.extend(pipeline_groups)
    return actions


def generate_launch_description() -> LaunchDescription:
    rf_gz_share = get_package_share_path("rf_gz")
    rf_dectectors_share = get_package_share_path("rf_dectectors")
    default_world_path = rf_gz_share / "worlds" / "demo_world.sdf"
    default_model_path = rf_dectectors_share / "resource" / "best.pt"

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world_path),
        DeclareLaunchArgument("model_path", default_value=default_model_path),
        DeclareLaunchArgument("conf_thresh", default_value="0.2"),
        DeclareLaunchArgument("stft_win_s", default_value="0.04915"),
        DeclareLaunchArgument("fft_size", default_value="2048"),
        DeclareLaunchArgument("hop_size", default_value="512"),
        OpaqueFunction(function=_launch_setup),
    ])
