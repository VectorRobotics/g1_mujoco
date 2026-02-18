import os
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # ros2 launch runs under system Python, so sys.executable is /usr/bin/python3
    # Use VIRTUAL_ENV env var to find the venv's Python instead
    venv = os.environ.get('VIRTUAL_ENV')
    python_exec = os.path.join(venv, 'bin', 'python3') if venv else sys.executable

    return LaunchDescription([
        ExecuteProcess(
            cmd=[python_exec, '-m', 'g1_mujoco.mujoco_test'],
            output='screen',
        ),
    ])
