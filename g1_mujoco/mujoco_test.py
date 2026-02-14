import time
import mujoco
import mujoco.viewer
from threading import Thread, Event
import threading
import rclpy # Import rclpy
from rclpy.node import Node # Import Node
from sensor_msgs.msg import JointState # Import JointState message type

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand
import config

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('mujoco_joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states', # Topic name
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(JointState, "feedback", 10)

        self.get_logger().info("Mujoco JointState Subscriber Node has started.")

        self.locker = threading.Lock()

        self.mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
        self.mj_data = mujoco.MjData(self.mj_model)

        if config.ENABLE_ELASTIC_BAND:
            self.elastic_band = ElasticBand()
            self.band_attached_link = self.mj_model.body("torso_link").id
            self.viewer = mujoco.viewer.launch_passive(
                self.mj_model, self.mj_data, key_callback=self.elastic_band.MujuocoKeyCallback
            )
        else:
            self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)

        self.mj_model.opt.timestep = config.SIMULATE_DT
        num_motor_ = self.mj_model.nu # overall DOF, should = 29 for g1
        dim_motor_sensor_ = 3 * num_motor_ # pos, vel, torque for each DOF

        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        self.unitree = UnitreeSdk2Bridge(self.mj_model, self.mj_data)

        if config.USE_JOYSTICK:
            self.unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
        if config.PRINT_SCENE_INFORMATION:
            self.unitree.PrintSceneInformation()

        self.simulation_thread = threading.Thread(target=self.SimulationThread, daemon=True)
        self.simulation_thread.start()

        self.physics_thread = threading.Thread(target=self.PhysicsViewerThread, daemon=True)
        self.physics_thread.start()


    def listener_callback(self, msg: JointState):
        self.exp_joint_states = msg

    def SimulationThread(self):
        while self.viewer.is_running():
            step_start = time.perf_counter()

            self.locker.acquire()

            if config.ENABLE_ELASTIC_BAND:
                if self.elastic_band.enable:
                    self.mj_data.xfrc_applied[self.band_attached_link, :3] = self.elastic_band.Advance(
                        self.mj_data.qpos[:3], self.mj_data.qvel[:3]
                    )
            
            self.unitree.LowCmdHandler_Joint(self.exp_joint_states)
            jointstate = JointState()
            
            jointstate.name = self.unitree.mj_model.joint_names
            jointstate.position = self.unitree.mj_data.sensordata[0:self.mj_model.nu]
            jointstate.velocity = self.unitree.mj_data.sensordata[self.mj_model.nu:2*self.mj_model.nu]

            self.publisher.publish(jointstate)




            mujoco.mj_step(self.mj_model, self.mj_data)

            self.locker.release()

            time_until_next_step = self.mj_model.opt.timestep - (
                time.perf_counter() - step_start
            )
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

        self.destroy_node()
        print("SimulationThread: Stopping ROS 2 node and spin thread...")


    def PhysicsViewerThread(self):
        while self.viewer.is_running():
            self.locker.acquire() # thread lock, mutex
            self.viewer.sync()
            self.locker.release()
            time.sleep(config.VIEWER_DT)

    def die(self): # stop_event.set()
        self.simulation_thread.join()
        self.physics_thread.join()

        if self.viewer.is_running():
            self.viewer.close()

        self.destroy_node()

def main():
    # Initialize ROS 2 once in the main thread
    rclpy.init(args=None)

    node = JointStateSubscriber()
    # Create a separate thread for spinning the ROS 2 node
    try:
        rclpy.spin(node)
    finally:
        node.die()
        rclpy.shutdown()
        print("\nAll threads stopped and resources cleaned up.")


   