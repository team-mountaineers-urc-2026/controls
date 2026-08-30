import os 
from glob import glob # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 
from setuptools import find_packages, setup # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 
package_name = 'controls_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='nathanpadkins@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can = controls_pkg.can:main',
            'motor = controls_pkg.motor:main',

            'drivetrain = controls_pkg.drivetrain:main',

            'manipulator = controls_pkg.manipulator:main',
            'science_manipulator = controls_pkg.science_manipulator:main', 
            'joint_control = controls_pkg.joint_control:main',
            'science_joint_control = controls_pkg.science_joint_control:main',
            'ik_control = controls_pkg.ik_control:main',
            'joy_mux = controls_pkg.joy_mux:main',
            'gimbal_joy_con = controls_pkg.gimbal_joy_con:main',
            'arm_zeroing = controls_pkg.five_dof_zeroing:main',
            'zero = controls_pkg.zero_node:main',

            'arm_replay = controls_pkg.autonomous_replay:main',
            'positional_replay = controls_pkg.positional_replay:main',
        
            'zero_ik_manipulator = controls_pkg.zero_ik_manipulator:main',
            'drivetrain_debugger = controls_pkg.drivetrain_debugger:main',
            'manipulator_debugger = controls_pkg.manipulator_debugger:main',
        ],
    },
)
