from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    iq_topic = LaunchConfiguration("iq_topic")
    fs_hz = LaunchConfiguration("fs_hz")
    center_freq_hz = LaunchConfiguration("center_freq_hz")
    stft_win_s = LaunchConfiguration("stft_win_s")
    fft_size = LaunchConfiguration("fft_size")
    hop_size = LaunchConfiguration("hop_size")
    model_path = LaunchConfiguration("model_path")
    conf_thresh = LaunchConfiguration("conf_thresh")
    stft_topic = LaunchConfiguration("stft_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    scores_topic = LaunchConfiguration("scores_topic")
    scoreboard_agg = LaunchConfiguration("scoreboard_agg")

    return LaunchDescription([
        DeclareLaunchArgument("iq_topic", default_value="/rf/rx1/iq"),
        DeclareLaunchArgument("fs_hz", default_value="10000000.0"),
        DeclareLaunchArgument("center_freq_hz", default_value="2400000000.0"),
        DeclareLaunchArgument("stft_win_s", default_value="0.04915"),
        DeclareLaunchArgument("fft_size", default_value="2048"),
        DeclareLaunchArgument("hop_size", default_value="512"),
        DeclareLaunchArgument("model_path", default_value="best.pt"),
        DeclareLaunchArgument("conf_thresh", default_value="0.2"),
        DeclareLaunchArgument("stft_topic", default_value="/stft_node/stft"),
        DeclareLaunchArgument("detections_topic", default_value="/yolo_detector_node/detections"),
        DeclareLaunchArgument("scores_topic", default_value="/scoreboard_node/scores"),
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
                "input_topic": iq_topic,
                "stft_win_s": stft_win_s,
            }],
        ),
        Node(
            package="rf_dectectors",
            executable="yolo_detector",
            name="yolo_detector_node",
            output="screen",
            parameters=[{
                "model_path": model_path,
                "conf_thresh": conf_thresh,
                "input_topic": stft_topic,
                "output_topic": detections_topic,
            }],
        ),
        Node(
            package="rf_cartography",
            executable="scoreboard",
            name="scoreboard_node",
            output="screen",
            parameters=[{
                "input_topic": detections_topic,
                "output_topic": scores_topic,
                "aggregation": scoreboard_agg,
            }],
        ),
    ])
