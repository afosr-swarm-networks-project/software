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
    """Build a ros_gz_sim create Node that spawns one RF model."""
    pose = instance.get("pose") or {}
    arguments = [
        "-string", urdf_str,
        "-x", str(pose.get("x", 0.0)),
        "-y", str(pose.get("y", 0.0)),
        "-z", str(pose.get("z", 0.0)),
        "-R", str(pose.get("roll",  0.0)),
        "-P", str(pose.get("pitch", 0.0)),
        "-Y", str(pose.get("yaw",   0.0)),
    ]
    if "name" in instance:
        arguments += ["-name", instance["name"]]

    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{label}",
        output="screen",
        arguments=arguments,
    )


def _robot_state_publisher_node(label: str, urdf_str: str, rsp_cfg) -> Node:
    """Build a robot_state_publisher Node."""
    extra = rsp_cfg if isinstance(rsp_cfg, dict) else {}
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"{label}_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_str, "use_sim_time": True, **extra}],
    )


def _static_transform_publisher_node(label: str, pose: dict, stp_cfg: dict) -> Node:
    """Build a tf2_ros static_transform_publisher Node from pose and config."""
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{label}_static_tf",
        output="screen",
        arguments=[
            "--frame-id",       stp_cfg["parent_frame"],
            "--child-frame-id", stp_cfg["child_frame"],
            "--x",     str(pose.get("x",     0.0)),
            "--y",     str(pose.get("y",     0.0)),
            "--z",     str(pose.get("z",     0.0)),
            "--roll",  str(pose.get("roll",  0.0)),
            "--pitch", str(pose.get("pitch", 0.0)),
            "--yaw",   str(pose.get("yaw",   0.0)),
        ],
    )


def _pipeline_group(pipeline_launch: Path, pipeline_name: str, pipeline_cfg: dict) -> GroupAction:
    """Build a pipeline GroupAction for one named pipeline entry."""
    return GroupAction([
        PushRosNamespace(pipeline_name),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pipeline_launch)),
            launch_arguments={
                **{k: str(v) for k, v in pipeline_cfg.items()},
                "use_sim_time": "true",
            }.items(),
        ),
    ])


def _create_models(urdf_dir: Path, models_cfg: dict) -> list:
    """Spawn all models and their optional robot_state_publisher / static_transform_publisher."""
    actions = []

    for xacro_file, instances in models_cfg.items():
        if not instances:
            continue
        for i, instance in enumerate(instances):
            instance = instance or {}
            args = instance.get("args") or {}
            file_stem = Path(Path(xacro_file).stem).stem  # strip both .urdf.xacro extensions
            label = instance.get("name") or f"{file_stem}_{i}"

            urdf_str = _load_model(urdf_dir, xacro_file, args)

            # Spawn the model
            actions.append(_spawn_node(label, instance, urdf_str))

            # Optional: robot_state_publisher
            rsp_cfg = instance.get("robot_state_publisher")
            if rsp_cfg:
                actions.append(_robot_state_publisher_node(label, urdf_str, rsp_cfg))

            # Optional: static_transform_publisher
            stp_cfg = instance.get("static_transform_publisher")
            if stp_cfg:
                if not isinstance(stp_cfg, dict) or "parent_frame" not in stp_cfg or "child_frame" not in stp_cfg:
                    print(f"[WARN] {label}: static_transform_publisher requires parent_frame and child_frame — skipping")
                else:
                    pose = instance.get("pose") or {}
                    actions.append(_static_transform_publisher_node(label, pose, stp_cfg))

    return actions


def _create_pipelines(pipeline_launch: Path, pipelines_cfg: list) -> list:
    """Create a GroupAction for each named pipeline entry."""
    actions = []
    for pipeline_item in (pipelines_cfg or []):
        for pipeline_name, pipeline_cfg in pipeline_item.items():
            actions.append(
                _pipeline_group(pipeline_launch, pipeline_name, pipeline_cfg or {})
            )
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

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    actions = [gz_sim, clock_bridge]
    actions.extend(_create_models(urdf_dir, config.get("models") or {}))
    actions.extend(_create_pipelines(pipeline_launch, config.get("pipelines") or []))
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
