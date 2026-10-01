"""Alternative ROS launch entry point; LAB_ROOT points at the repository."""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    root = os.environ['LAB_ROOT']
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='--headless'),
        ExecuteProcess(cmd=['/usr/bin/python3', root+'/scripts/supervise.py', LaunchConfiguration('mode')], output='screen')
    ])
