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

        ## ROS interfaces — topic depends on test mode
        sub_topics = {
            "position": "position_control",
            "effort": "effort_control",
        }
        self.joint_sub = self.create_subscription(JointState, sub_topics[config.TEST_MODE], self.listener_callback, 10)

        # Feedback publisher (needed by Impedance Control and Visual Servo)
        self.joint_pub = self.create_publisher(JointState, "feedback", 10)

        ## MuJoCo model and data
        self.model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
        self.data = mujoco.MjData(self.model)

        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.viewer.cam.lookat = [-0.016, -0.015, 0.68]
        self.viewer.cam.distance = 1.74
        self.viewer.cam.azimuth = 144.18
        self.viewer.cam.elevation = -21.56

        # PD control parameters
        self.arm_kp = 10
        self.arm_kd = 2
        self.arm_ki = 0.01
        self.error_int = 0

        self.arm_ctrl_joint_states = None
        self.wb_fdbk_joint_states = JointState()
        self.wb_joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]
        self.wb_fdbk_joint_states.name = self.wb_joint_names

        # Single thread for physics + viewer (mjData is NOT thread-safe)
        self.sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self.sim_thread.start()

        self.get_logger().info("Mujoco simulation node has started.")
        self.PrintSceneInformation()

    def listener_callback(self, msg: JointState):
        self.arm_ctrl_joint_states = msg

    def _sim_loop(self):
        """Single-thread loop: sub-step physics with control at every step,
        sync viewer once per frame.

        mjData is not thread-safe, so physics and viewer.sync() must be
        in the same thread. Control is applied at every sub-step to maintain
        stability regardless of VIEWER_DT.
        """

        self.model.opt.timestep = config.SIMULATE_DT
        n_steps = max(1, int(config.VIEWER_DT / self.model.opt.timestep))

        while self.viewer.is_running():
            frame_start = time.perf_counter()

            # Sub-step: control + physics at every timestep
            for _ in range(n_steps):
                if self.arm_ctrl_joint_states is not None:
                    if config.TEST_MODE == "effort":
                        self.apply_torques(self.arm_ctrl_joint_states)
                    elif config.TEST_MODE == "position":
                        self.control_arm(self.arm_ctrl_joint_states)
                mujoco.mj_step(self.model, self.data)

            # Publish feedback (once per frame, after sub-steps)
            self.wb_fdbk_joint_states.position = self.data.qpos[:self.model.njnt].tolist()
            self.wb_fdbk_joint_states.velocity = self.data.qvel[:self.model.njnt].tolist()
            self.wb_fdbk_joint_states.effort = self.data.actuator_force[:self.model.njnt].tolist()
            self.joint_pub.publish(self.wb_fdbk_joint_states)

            # Sync viewer (once per frame)
            self.viewer.sync()

            # Sleep to maintain real-time
            elapsed = time.perf_counter() - frame_start
            sleep_time = config.VIEWER_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.get_logger().info("Viewer closed, shutting down...")
        rclpy.try_shutdown()

    def apply_torques(self, msg: JointState):
        """Impedance Control mode: directly apply torques from the controller."""
        for i, joint_name in enumerate(msg.name):
            act_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name[:-6])
            if act_id >= 0:
                self.data.ctrl[act_id] = msg.effort[i]

    def control_arm(self, msg: JointState):
        """IK mode: effort feedforward + PD position/velocity tracking."""
        for i, joint_name in enumerate(msg.name):
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            act_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name[:-6])

            if jnt_id >= 0 and act_id >=0:
                self.error_int += (msg.position[i] - self.data.qpos[jnt_id])
                self.data.ctrl[act_id] = (
                    msg.effort[i]
                    + self.arm_kp * (msg.position[i] - self.data.qpos[jnt_id])
                    + self.arm_kd * (msg.velocity[i] - self.data.qvel[jnt_id])
                    + self.arm_ki * self.error_int
                )

    def PrintSceneInformation(self):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i
            )
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.model.nsensor):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_SENSOR, i
            )
            if name:
                print(
                    "sensor_index:",
                    index,
                    ", name:",
                    name,
                    ", dim:",
                    self.model.sensor_dim[i],
                )
            index = index + self.model.sensor_dim[i]
        print(" ")


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
