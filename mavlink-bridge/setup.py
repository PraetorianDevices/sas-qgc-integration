import os

from setuptools import setup, find_packages

package_name = 'mavlink-bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'mavlink_v2',
        'gps_spoof_mavlink_bridge',
        'telemetry_mavlink_bridge',
        'mission_control_bridge',
        'fleet_manager_mavlink_bridge',
        'collision_mavlink_bridge',
        'emergency_wipe_mavlink_bridge',
        'mavlink_router_node',
        'test_gps_spoof_alert_generator',
    ],
    # Without this block, colcon installs the package with no ament_index
    # resource_index marker and no launch files copied to share/ -- `ros2
    # launch mavlink-bridge <file>.py` cannot find either the package or the
    # launch file after install. Matches the convention SAS/setup.py already
    # uses. Launch files live at this package's root (not a launch/
    # subdirectory), so they're installed explicitly by name rather than via
    # a launch/*.py glob.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            ['launch_sas_qgc_integration.py', 'launch_gps_spoof_qgc.py']),
    ],
    install_requires=[
        'rclpy',
        'std_msgs',
        'std_srvs',
    ],
    entry_points={
        'console_scripts': [
            'gps_spoof_mavlink_bridge = gps_spoof_mavlink_bridge:main',
            'telemetry_mavlink_bridge = telemetry_mavlink_bridge:main',
            'mission_control_bridge = mission_control_bridge:main',
            'fleet_manager_mavlink_bridge = fleet_manager_mavlink_bridge:main',
            'collision_mavlink_bridge = collision_mavlink_bridge:main',
            'emergency_wipe_mavlink_bridge = emergency_wipe_mavlink_bridge:main',
            'mavlink_router_node = mavlink_router_node:main',
            'test_gps_spoof_alert_generator = test_gps_spoof_alert_generator:main',
        ],
    },
)
