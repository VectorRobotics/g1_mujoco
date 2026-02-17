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

SIMULATE_DT = 1/200  # Physics timestep (small for stability) 0.005s
VIEWER_DT = 1.0/25.0  # ~25 fps for viewer 0.04s

# test mode: IK, Impedance Control, Visual Servo
# TEST_MODE = _os.environ.get('G1_TEST_MODE', 'Impedance Control')
TEST_MODE = "Impedance Control"