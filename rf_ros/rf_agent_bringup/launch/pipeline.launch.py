from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    iq_topic = LaunchConfiguration("iq_topic")
    fs_hz = LaunchConfiguration("fs_hz")
    center_freq_hz = LaunchConfiguration("center_freq_hz")
    stft_win_s = LaunchConfiguration("stft_win_s")
    fft_size = LaunchConfiguration("fft_size")
    hop_size = LaunchConfiguration("hop_size")
    model_path = LaunchConfiguration("model_path")
    conf_thresh = LaunchConfiguration("conf_thresh")
    scores_topic = LaunchConfiguration("scores_topic")
    scoreboard_agg = LaunchConfiguration("scoreboard_agg")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("iq_topic", default_value="iq"),
        DeclareLaunchArgument("scores_topic", default_value="scores"),
        DeclareLaunchArgument("fs_hz", default_value="10000000.0"),
        DeclareLaunchArgument("center_freq_hz", default_value="2400000000.0"),
        DeclareLaunchArgument("stft_win_s", default_value="0.04915"),
        DeclareLaunchArgument("fft_size", default_value="2048"),
        DeclareLaunchArgument("hop_size", default_value="512"),
        DeclareLaunchArgument("model_path", default_value="best.pt"),
        DeclareLaunchArgument("conf_thresh", default_value="0.2"),
        DeclareLaunchArgument("scoreboard_agg", default_value="max"),
        Node(
            package="rf_signal_proc",
            executable="stft_node.py",
            name="stft_node",
            output="screen",
            parameters=[{
                "fs_hz": fs_hz,
                "center_freq_hz": center_freq_hz,
                "fft_size": fft_size,
                "hop_size": hop_size,
                "stft_win_s": stft_win_s,
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("iq", iq_topic),
            ]
        ),
        Node(
            package="rf_dectectors",
            executable="yolo_detector",
            name="yolo_detector_node",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "conf_thresh": conf_thresh,
                "use_sim_time": use_sim_time,
            }],
        ),
        Node(
            package="rf_cartography",
            executable="scoreboard",
            name="scoreboard_node",
            output="screen",
            parameters=[{
                "aggregation": scoreboard_agg,
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("scores", scores_topic),
            ]
        ),
    ])
