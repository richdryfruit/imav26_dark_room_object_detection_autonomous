from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'systemd'), glob(os.path.join(package_name, '*.service'))),
        (os.path.join('share', package_name, 'sim'), glob(os.path.join('sim', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ark-jetson-orin',
    maintainer_email='ishan.aphanse2807@gmail.com',
    description='Pixhawk telemetry reader and offboard launch',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pixhawk_node = drone_testing.pixhawk_node:main',
            'preflight_check = drone_testing.preflight_check:main',
            'offboard_mission = drone_testing.offboard_mission:main',
            'offboard_takeoff = drone_testing.offboard_takeoff:main',
            'offboard_translate = drone_testing.offboard_translate:main',
            'offboard_sequence = drone_testing.offboard_sequence:main',
            'offboard_sequence_vio = drone_testing.offboard_sequence_vio:main',
            'lcd_status = drone_testing.lcd_status:main',
            'zed_localization = drone_testing.zed_localization:main',
            'cam = drone_testing.cam:main',
            'fc_reboot = drone_testing.fc_reboot:main',
            'window_detect = drone_testing.window_detect:main',
            'window_scan = drone_testing.window_scan:main',
            'window_traverse = drone_testing.window_traverse:main',
            'window_room_traverse = drone_testing.window_room_traverse:main',
            'doll_detect = drone_testing.doll_detect:main',
            'room_display = drone_testing.room_display:main',
            'qgc_doll_status = drone_testing.qgc_doll_status:main',
            'doll_count_gui = drone_testing.doll_count_gui:main',
            'aruco_pose = drone_testing.aruco_pose:main',
            'ring_detect = drone_testing.ring_detect:main',
            'ring_grab = drone_testing.ring_grab:main',
            'precision_land = drone_testing.precision_land:main',
            'mission_fsm = drone_testing.mission_fsm:main',
            'mission_fsm_part1 = drone_testing.mission_fsm_part1:main',
            'mission_fsm_part2 = drone_testing.mission_fsm_part2:main',
            'mission_fsm_full = drone_testing.mission_fsm_full:main',
            'mission_fsm_darkroom_backup = drone_testing.mission_fsm_darkroom_backup:main',
            'sim_operator = drone_testing.sim_operator:main',
            'yaw_seed = drone_testing.yaw_seed:main',
            'floor_line = drone_testing.floor_line:main',
            'calibrate_camera = drone_testing.calibrate_camera:main',
            'doll_report_text = drone_testing.doll_report_text:main',
        ],
    },
)
