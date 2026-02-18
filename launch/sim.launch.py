import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Map mode argument to controller node
CONTROLLER_NODES = {
    'impedance': ('g1_pilot', 'grav_ff'),
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

    # target_pose: [x, y, z, qw, qx, qy, qz] — only used in visual_servo mode
    target_pose_arg = DeclareLaunchArgument(
        'target_pose',
        default_value='[0.6, -0.4, 0.2, 1.0, 0.0, 0.0, 0.0]',
        description='Target pose [x, y, z, qw, qx, qy, qz] for visual_servo mode'
    )

    # Resolve venv Python
    venv = os.environ.get('VIRTUAL_ENV')
    python_exec = os.path.join(venv, 'bin', 'python3') if venv else sys.executable

    # Use OpaqueFunction to defer until launch time for dict lookups.
    from launch.actions import OpaqueFunction
    import ast

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

        # For visual_servo mode, send an action goal after a delay
        if mode == 'visual_servo':
            pose = ast.literal_eval(context.launch_configurations['target_pose'])
            x, y, z, qw, qx, qy, qz = pose

            goal_yaml = (
                f"{{target_pose: {{pose: {{"
                f"position: {{x: {x}, y: {y}, z: {z}}}, "
                f"orientation: {{w: {qw}, x: {qx}, y: {qy}, z: {qz}}}"
                f"}}}}}}"
            )

            actions.append(TimerAction(
                period=5.0,  # wait for nodes to initialize and FK to compute
                actions=[ExecuteProcess(
                    cmd=[
                        'ros2', 'action', 'send_goal',
                        '/move_arm', 'g1_pilot/action/MoveArm',
                        goal_yaml, '--feedback',
                    ],
                    output='screen',
                )],
            ))

        return actions

    return LaunchDescription([
        mode_arg,
        target_pose_arg,
        OpaqueFunction(function=launch_nodes),
    ])
