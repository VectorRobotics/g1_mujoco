import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Map mode argument to controller node
CONTROLLER_NODES = {
    'impedance': ('g1_pilot', 'impede_arms'),
    'visual_servo': ('g1_pilot', 'visual_servo'),
    'ik': ('g1_pilot', 'ik_joint_state_publisher'),
}

# Map mode argument to G1_TEST_MODE env var value
MODE_ENV = {
    'impedance': 'Impedance Control',
    'visual_servo': 'Visual Servo',
    'ik': 'IK',
}


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        choices=list(CONTROLLER_NODES.keys()),
        description='Test mode: impedance, visual_servo, or ik (REQUIRED)'
    )

    # Resolve venv Python
    venv = os.environ.get('VIRTUAL_ENV')
    python_exec = os.path.join(venv, 'bin', 'python3') if venv else sys.executable

    # We need to resolve mode at generate time since ExecuteProcess env
    # doesn't support substitutions well for dict lookups.
    # Use OpaqueFunction to defer until launch time.
    from launch.actions import OpaqueFunction

    def launch_nodes(context):
        mode = context.launch_configurations['mode']
        pkg, exe = CONTROLLER_NODES[mode]
        test_mode_env = MODE_ENV[mode]

        actions = []

        # MuJoCo simulator with test mode set via env var
        actions.append(ExecuteProcess(
            cmd=[python_exec, '-m', 'g1_mujoco.mujoco_test'],
            additional_env={'G1_TEST_MODE': test_mode_env},
            output='screen',
        ))

        # Controller node
        actions.append(Node(
            package=pkg,
            executable=exe,
            output='screen',
        ))

        return actions

    return LaunchDescription([
        mode_arg,
        OpaqueFunction(function=launch_nodes),
    ])
