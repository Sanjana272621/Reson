"""
Launches the full reson pipeline: CSV replay -> FFT -> diagnosis.

Usage:
  ros2 launch reson reson_pipeline.launch.py csv_path:=/path/to/fault2_1.csv
  ros2 launch reson reson_pipeline.launch.py csv_path:=/path/to/normal_2.csv loop:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('reson')
    default_params = os.path.join(pkg_share, 'config', 'reson_params.yaml')
    default_csv = os.path.join(pkg_share, 'data', 'normal_2.csv')

    csv_path_arg = DeclareLaunchArgument(
        'csv_path', default_value=default_csv,
        description='Path to the vibration CSV to replay')
    loop_arg = DeclareLaunchArgument(
        'loop', default_value='false',
        description='Loop the CSV replay indefinitely')
    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate', default_value='1.0',
        description='Replay speed multiplier (2.0 = twice as fast)')
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='YAML file with fft_node / diagnosis_node parameters '
                     '(window size, baseline fingerprint, thresholds, ...)')

    csv_replay_node = Node(
        package='reson',
        executable='csv_replay_node',
        name='csv_replay_node',
        output='screen',
        parameters=[{
            'csv_path': LaunchConfiguration('csv_path'),
            'loop': LaunchConfiguration('loop'),
            'playback_rate': LaunchConfiguration('playback_rate'),
        }],
    )

    fft_node = Node(
        package='reson',
        executable='fft_node',
        name='fft_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    diagnosis_node = Node(
        package='reson',
        executable='diagnosis_node',
        name='diagnosis_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        csv_path_arg,
        loop_arg,
        playback_rate_arg,
        params_file_arg,
        csv_replay_node,
        fft_node,
        diagnosis_node,
    ])
