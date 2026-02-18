import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'g1_mujoco'

# Collect all asset files, preserving directory structure
asset_data_files = []
for root, dirs, files in os.walk('assets'):
    if files:
        install_dir = os.path.join('share', package_name, root)
        asset_data_files.append((install_dir, [os.path.join(root, f) for f in files]))

setup(
 name=package_name,
 version='0.0.0',
 packages=find_packages(exclude=['test']),
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
   ] + asset_data_files,
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Jinyao Zhou',
 maintainer_email='kaleidojean@gmail.com',
 description='Package for G1 mujoco simulation',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'run_sim = g1_mujoco.mujoco_test:main'
     ],
   },
)