import threading
import numpy as np
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

from . import config

JOINT_NAMES = [
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
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        ## MuJoCo model and data
        self.model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
        self.data = mujoco.MjData(self.model)

        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.viewer.cam.lookat = [-0.016, -0.015, 0.68]
        self.viewer.cam.distance = 1.74
        self.viewer.cam.azimuth = 144.18
        self.viewer.cam.elevation = -21.56

        # Joint ID mapping & validation
        self.act_id = np.array([mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint[:-6]) for joint in JOINT_NAMES])
        self.jnt_id = np.array([mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint) for joint in JOINT_NAMES])

        # FIX: Check >= 0 to avoid dropping joint ID 0
        self.mask = (self.act_id >= 0) & (self.jnt_id >= 0)

        self.act_id = self.act_id[self.mask]
        self.jnt_id = self.jnt_id[self.mask]
        self.joint_names = [JOINT_NAMES[i] for i in range(len(JOINT_NAMES)) if self.mask[i]]

        # MuJoCo qpos/qvel indices for the selected joints
        self.qpos_idx = np.array([self.model.jnt_qposadr[jid] for jid in self.jnt_id])
        self.qvel_idx = np.array([self.model.jnt_dofadr[jid] for jid in self.jnt_id])

        # Gains
        # self.joint_kp = np.array([
        #     250.0, 250.0, 250.0,
        #     100.0, 100.0, 100.0, 100.0, 20.0, 20.0, 20.0,
        #     100.0, 100.0, 100.0, 100.0, 20.0, 20.0, 20.0
        # ])[self.mask]
        # self.joint_kd = np.array([
        #     5.0, 5.0, 5.0,
        #     5.0, 5.0, 2.0, 2.0, 2.0, 2.0, 2.0,
        #     5.0, 5.0, 2.0, 2.0, 2.0, 2.0, 2.0
        # ])[self.mask]
        # self.joint_ki = np.array([
        #     0.0, 0.0, 0.0,
        #     0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15,
        #     0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15,
        # ])[self.mask]

        self.joint_kp = np.array([
            40.0, 40.0, 40.0,
            12.0, 12.0, 10.0, 10.0, 6.0, 6.0, 6.0,
            12.0, 12.0, 10.0, 10.0, 6.0, 6.0, 6.0,
        ])[self.mask]

        self.joint_kd = np.array([
            5.0, 5.0, 5.0,
            2.5, 2.5, 2.0, 2.0, 1.0, 1.0, 1.0,
            2.5, 2.5, 2.0, 2.0, 1.0, 1.0, 1.0,
        ])[self.mask]

        self.joint_ki = np.array([
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ])[self.mask]

        # self.joint_kp = np.array([
        #     150.0, 150.0, 150.0,
        #     100.0, 100.0, 18.0, 18.0, 18.0, 18.0, 18.0,
        #     100.0, 100.0, 18.0, 18.0, 18.0, 18.0, 18.0,
        # ])[self.mask]
        # self.joint_kd = np.array([
        #     2.0, 2.0, 2.0,
        #     5.0, 5.0, 2.0, 2.0, 2.0, 2.0, 2.0,
        #     5.0, 5.0, 2.0, 2.0, 2.0, 2.0, 2.0,
        # ])[self.mask]
        # self.joint_ki = np.array([
        #     0.0, 0.0, 0.0,
        #     0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15,
        #     0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15,
        # ])[self.mask]*0.0

        # FIX: PID state as NumPy arrays matching masked length
        num_valid_joints = len(self.act_id)
        self.prev_error = np.zeros(num_valid_joints)
        self.integral = np.zeros(num_valid_joints)
        self.d_term = np.zeros(num_valid_joints)
        self.Tf = 0.1

        self.arm_ctrl_joint_states = None
        self.wb_fdbk_joint_states = JointState()
        self.wb_fdbk_joint_states.name = self.joint_names
        self.clock_msg = Clock()

        # Single thread for physics + viewer (mjData is NOT thread-safe)
        self.sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self.sim_thread.start()

        self.get_logger().info("Mujoco simulation node has started.")

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

            # Sub-step: control + physics at every timestep
            for _ in range(n_steps):
                if self.arm_ctrl_joint_states is not None:
                    if config.TEST_MODE == "effort":
                        self.apply_torques(self.arm_ctrl_joint_states)
                    elif config.TEST_MODE == "position":
                        self.control_arm(self.arm_ctrl_joint_states)
                mujoco.mj_step(self.model, self.data)

            seconds = self.data.time
            self.clock_msg.clock.sec = int(seconds)
            self.clock_msg.clock.nanosec = int((seconds - int(seconds)) * 1e9)
            self.clock_pub.publish(self.clock_msg)

            # Publish feedback (once per frame, after sub-steps)
            self.wb_fdbk_joint_states.header.stamp = self.clock_msg.clock
            self.wb_fdbk_joint_states.position = self.data.qpos[self.qpos_idx].tolist()
            self.wb_fdbk_joint_states.velocity = self.data.qvel[self.qvel_idx].tolist()
            self.wb_fdbk_joint_states.effort = self.data.actuator_force[self.act_id].tolist()
            self.joint_pub.publish(self.wb_fdbk_joint_states)

            # Sync viewer (once per frame)
            self.viewer.sync()

        self.get_logger().info("Viewer closed, shutting down...")
        rclpy.try_shutdown()

    def apply_torques(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        cmd_efforts = np.array([msg.effort[name_to_idx[name]] if name in name_to_idx else 0.0 for name in self.joint_names])
        self.data.ctrl[self.act_id] = cmd_efforts

    def control_arm(self, msg: JointState):
        """PID with Tustin (bilinear) discretization and filtered D term.

        Transfer function:
                        Ts*(z+1)                    1
          u = Kp*e + Ki*--------*e + Kd * ----------------------- * e
                        2*(z-1)           Tf + Ts/2*(z+1)/(z-1)
        """
        Ts = config.SIMULATE_DT
        Tf = self.Tf

        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        des_pos = np.array([msg.position[name_to_idx[name]] if name in name_to_idx and len(msg.position) > name_to_idx[name] else 0.0 for name in self.joint_names])
        des_vel = np.array([msg.velocity[name_to_idx[name]] if name in name_to_idx and len(msg.velocity) > name_to_idx[name] else 0.0 for name in self.joint_names])
        des_eff = np.array([msg.effort[name_to_idx[name]] if name in name_to_idx and len(msg.effort) > name_to_idx[name] else 0.0 for name in self.joint_names])

        error = des_pos - self.data.qpos[self.qpos_idx]

        self.integral += self.joint_ki * Ts / 2.0 * (error + self.prev_error)
        self.integral = np.clip(self.integral, -60.0, 60.0)

        self.d_term = (
            self.joint_kd * (error - self.prev_error) + (Tf - Ts / 2.0) * self.d_term
        ) / (Tf + Ts / 2.0)

        effort = des_eff + self.joint_kp * error + self.integral + self.d_term

        self.prev_error = error
        self.data.ctrl[self.act_id] = effort


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