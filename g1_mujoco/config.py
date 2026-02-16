import os as _os
from ament_index_python.packages import get_package_share_directory as _get_pkg_share

ROBOT = "g1"
_SHARE_DIR = _get_pkg_share('g1_mujoco')
ROBOT_SCENE = _os.path.join(_SHARE_DIR, "assets", ROBOT, "scene.xml")
DOMAIN_ID = 1 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = 0 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.005  # Physics timestep (small for stability)
# VIEWER_DT = 1.0/30.0  # ~60 fps for viewer

# test mode: IK, Impedance Control, 
TEST_MODE = "Impedance Control"