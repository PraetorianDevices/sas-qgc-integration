from setuptools import setup, find_packages

setup(
    name='mavlink-bridge',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    py_modules=['gps_spoof_mavlink_bridge'],
    install_requires=[
        'rclpy',
        'std_msgs',
    ],
    entry_points={
        'console_scripts': [
            'gps_spoof_mavlink_bridge = gps_spoof_mavlink_bridge:main',
            'test_gps_spoof_alert_generator = test_gps_spoof_alert_generator:main',
        ],
    },
)
