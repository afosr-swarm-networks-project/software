import yaml
import xacro

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from pathlib import Path


def _load_config(config_path: Path) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f) or {}


def _load_model(urdf_dir: Path, xacro_file: str, args: dict) -> str:
    """Process a xacro file with the given args and return the URDF string."""
    xacro_path = urdf_dir / xacro_file
    mappings = {k: str(v) for k, v in args.items()}
    doc = xacro.process_file(str(xacro_path), mappings=mappings)
    return doc.toxml()


def _spawn_node(label: str, instance: dict, urdf_str: str) -> Node:
    """Build a ros_gz_sim create Node that spawns one model."""
    pose = instance.get("pose") or {}
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{label}",
        output="screen",
        arguments=[
            "-string", urdf_str,
            "-name",   label,
            "-x",      str(pose.get("x",     0.0)),
            "-y",      str(pose.get("y",     0.0)),
            "-z",      str(pose.get("z",     0.0)),
            "-R",      str(pose.get("roll",  0.0)),
            "-P",      str(pose.get("pitch", 0.0)),
            "-Y",      str(pose.get("yaw",   0.0)),
        ],
    )


def _create_rsp(urdf_str: str, rsp_cfg: dict) -> Node:
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_str, "use_sim_time": True, **rsp_cfg}],
    )


def _create_static_tfs(stp_cfgs: list) -> list:
    nodes = []
    for i, stp in enumerate(stp_cfgs):
        if "parent_frame" not in stp or "child_frame" not in stp:
            continue
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_tf_{i}",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "--frame-id",       stp["parent_frame"],
                "--child-frame-id", stp["child_frame"],
                "--x",     str(stp.get("x",     0.0)),
                "--y",     str(stp.get("y",     0.0)),
                "--z",     str(stp.get("z",     0.0)),
                "--roll",  str(stp.get("roll",  0.0)),
                "--pitch", str(stp.get("pitch", 0.0)),
                "--yaw",   str(stp.get("yaw",   0.0)),
            ],
        ))
    return nodes


def _create_pipelines(pipeline_cfgs: list, pipeline_launch: Path) -> list:
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pipeline_launch)),
            launch_arguments={
                **{k: str(v) for k, v in p.items()},
                "use_sim_time": "true",
            }.items(),
        )
        for p in pipeline_cfgs
    ]


def _ros_group(label: str, urdf_str: str, ros_cfg: dict,
               pipeline_launch: Path) -> GroupAction:
    """Wrap all ROS integration nodes for one model in a shared namespace."""
    actions = []

    if "robot_state_publisher" in ros_cfg:
        actions.append(_create_rsp(urdf_str, ros_cfg["robot_state_publisher"]))
    if "static_transform_publisher" in ros_cfg:
        actions.extend(_create_static_tfs(ros_cfg["static_transform_publisher"]))
    if "pipeline" in ros_cfg:
        actions.extend(_create_pipelines(ros_cfg["pipeline"], pipeline_launch))

    return GroupAction([PushRosNamespace(label), *actions])


def _create_nodes(urdf_dir: Path, pipeline_launch: Path, models_cfg: dict) -> list:
    """Spawn all models and launch their optional ROS integration."""
    actions = []

    for label, instance in (models_cfg or {}).items():
        instance = instance or {}
        model_cfg = instance.get("model") or {}
        xacro_file = model_cfg.get("file", "")
        args = model_cfg.get("args") or {}

        urdf_str = _load_model(urdf_dir, xacro_file, args)
        actions.append(_spawn_node(label, instance, urdf_str))

        ros_cfg = instance.get("ros")
        if ros_cfg:
            actions.append(_ros_group(label, urdf_str, ros_cfg, pipeline_launch))

    return actions


def _launch_setup(context, *args, **kwargs):
    ros_gz_sim_share       = get_package_share_path("ros_gz_sim")
    rf_agent_bringup_share = get_package_share_path("rf_agent_bringup")
    rf_gz_share            = get_package_share_path("rf_gz")

    world_path      = Path(LaunchConfiguration("world").perform(context))
    config_path     = Path(LaunchConfiguration("config").perform(context))
    urdf_dir        = Path(rf_gz_share) / "urdf"
    pipeline_launch = Path(rf_agent_bringup_share) / "launch" / "pipeline.launch.py"

    config = _load_config(config_path)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_sim_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {world_path}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    bridge_cfg_name = config.get("bridge_config")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge",
        output="screen",
        parameters=[{
            "config_file": str(config_path.parent / bridge_cfg_name),
            "use_sim_time": True,
        }],
    ) if bridge_cfg_name else None

    actions = [gz_sim]
    if bridge:
        actions.append(bridge)
    actions.extend(_create_nodes(urdf_dir, pipeline_launch, config.get("models")))
    return actions


def generate_launch_description() -> LaunchDescription:
    rf_gz_share         = get_package_share_path("rf_gz")
    default_world_path  = rf_gz_share / "worlds" / "demo_world.sdf"
    default_config_path = rf_gz_share / "config"  / "demo_config.yaml"

    return LaunchDescription([
        DeclareLaunchArgument("world",  default_value=str(default_world_path)),
        DeclareLaunchArgument("config", default_value=str(default_config_path)),
        OpaqueFunction(function=_launch_setup),
    ])
