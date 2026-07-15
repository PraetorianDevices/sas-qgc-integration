from setuptools import setup, find_packages

setup(
    name='mavlink-bridge',
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
