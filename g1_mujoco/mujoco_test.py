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

        # Per-joint PID gains: waist (3) + left arm (7) + right arm (7)
        _joint_names = [
            # Waist
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            # Left arm
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            # Right arm
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
        ]
        _kp_vals = [
            10.0, 150.0, 150.0,
            14.2, 14.2, 14.2, 14.2, 14.2, 16.7, 16.7,
            14.2, 14.2, 14.2, 14.2, 14.2, 16.7, 16.7,
        ]
        _kd_vals = [
            2.0, 3.5, 2.0,
            2.5, 2.5, 2.5, 3.5, 1.5, 1.5, 1.06,
            2.5, 2.5, 2.5, 3.5, 1.5, 1.5, 1.06,
        ]
        _ki_vals = [
            0.0, 0.0, 0.0,
            0.03, 0.056, 0.056, 0.056, 0.056, 0.056, 0.056,
            0.03, 0.056, 0.056, 0.056, 0.056, 0.056, 0.056,
        ]
        self.joint_kp = dict(zip(_joint_names, _kp_vals))
        self.joint_kd = dict(zip(_joint_names, _kd_vals))
        self.joint_ki = dict(zip(_joint_names, _ki_vals))
        self.integral_error = {name: 0.0 for name in _joint_names}

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

        print("Simulation running...")

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
        """PID position/velocity tracking with per-joint gains."""
        dt = config.SIMULATE_DT
        for i, joint_name in enumerate(msg.name):
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            act_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name[:-6])

            if jnt_id < 0 or act_id < 0:
                continue

            kp = self.joint_kp.get(joint_name, 10.0)
            kd = self.joint_kd.get(joint_name, 2.0)
            ki = self.joint_ki.get(joint_name, 0.0)

            error = msg.position[i] - self.data.qpos[jnt_id]

            # Accumulate integral only within deadband
            if abs(error) >= 0.001 and abs(error) < 0.4:
                self.integral_error[joint_name] = self.integral_error.get(joint_name, 0.0) + error

            # Clamp integral error
            self.integral_error[joint_name] = max(-1000.0, min(1000.0, self.integral_error[joint_name]))

            # Add integral term scaled by timestep, clamp to 50% of max motor torque
            adj = max(-60.0, min(60.0, msg.effort[i] + ki * self.integral_error[joint_name] * dt))

            # PD + feedforward
            effort = (
                adj
                + kp * error
                + kd * (msg.velocity[i] - self.data.qvel[jnt_id])
            )

            self.data.ctrl[act_id] = effort

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
