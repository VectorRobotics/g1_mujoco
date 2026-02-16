import time
import threading
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from . import config


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_joint_state_subscriber')

        ## ROS interfaces
        if config.TEST_MODE == "Impedance Control":
            self.joint_sub = self.create_subscription(JointState, 'controller/joint_states', self.listener_callback, 10)
            self.joint_pub = self.create_publisher(JointState, "feedback", 10)
        elif config.TEST_MODE == "IK":
            self.joint_sub = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)

        ## MuJoCo model and data
        self.model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
        self.data = mujoco.MjData(self.model)

        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # PD control parameters
        self.arm_kp = 10
        self.arm_kd = 2

        self.exp_joint_states = None
        self.feedback_joint_indices = None  # Built on first controller message

        # Single thread for physics + viewer (mjData is NOT thread-safe)
        self.sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self.sim_thread.start()

        self.get_logger().info("Mujoco simulation node has started.")

    def listener_callback(self, msg: JointState):
        # On first message, build the name→sensor index mapping
        if self.feedback_joint_indices is None:
            self.feedback_joint_indices = []
            for name in msg.name:
                idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                self.feedback_joint_indices.append(idx)
            self.feedback_joint_names = list(msg.name)
            self.get_logger().info(f"Feedback joints locked: {self.feedback_joint_names}")
        self.exp_joint_states = msg

    def _sim_loop(self):
        """Single-thread loop: sub-step physics with control at every step,
        sync viewer once per frame.

        mjData is not thread-safe, so physics and viewer.sync() must be
        in the same thread. Control is applied at every sub-step to maintain
        stability regardless of VIEWER_DT.
        """

        while self.viewer.is_running():
            frame_start = time.perf_counter()

            # Sub-step: control + physics at every timestep
            if self.exp_joint_states is not None:
                if config.TEST_MODE == "Impedance Control":
                    self.apply_torques(self.exp_joint_states)

                    # Publish feedback: only controller joints, same order
                    joint_state = JointState()
                    joint_state.name = self.feedback_joint_names
                    joint_state.position = [float(self.data.sensordata[idx]) for idx in self.feedback_joint_indices]
                    joint_state.velocity = [float(self.data.sensordata[idx + self.model.nu]) for idx in self.feedback_joint_indices]
                    self.joint_pub.publish(joint_state)

                elif config.TEST_MODE == "IK":
                    self.control_arm(self.exp_joint_states)
            mujoco.mj_step(self.model, self.data)

            # Sync viewer (once per frame, after all sub-steps)
            self.viewer.sync()

            # Sleep to maintain real-time
            elapsed = time.perf_counter() - frame_start
            sleep_time = config.SIMULATE_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.get_logger().info("Viewer closed, shutting down...")
        rclpy.try_shutdown()

    def apply_torques(self, msg: JointState):
        """Impedance Control mode: directly apply torques from the controller."""
        for i, joint_name in enumerate(msg.name):
            try:
                motor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                self.data.ctrl[motor_id] = msg.effort[i]
            except Exception as e:
                print(f"Error setting torque for {joint_name}: {e}")

    def control_arm(self, msg: JointState):
        for i, joint_name in enumerate(msg.name):
            try:
                motor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                self.data.ctrl[motor_id] = (
                    msg.effort[i] +
                    self.arm_kp * (msg.position[i] - self.data.sensordata[motor_id])
                    + self.arm_kd * (msg.velocity[i] - self.data.sensordata[motor_id + self.model.nu])
                )
            except KeyError:
                print(f"Warning: Joint '{joint_name}' not found in Mujoco model.")
            except Exception as e:
                print(f"Error setting motor command for {joint_name}: {e}")


def main():
    rclpy.init(args=None)
    node = MujocoSimNode()

    try:
        rclpy.spin(node)
    finally:
        if node.viewer.is_running():
            node.viewer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nAll threads stopped and resources cleaned up.")


if __name__ == "__main__":
    main()
