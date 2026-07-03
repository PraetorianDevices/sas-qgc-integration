#!/usr/bin/env python3
"""
Test Utility: GPS Spoofing Alert Generator

Publishes synthetic GPS spoofing alerts to /gps_spoof_alert for testing
the MAVLink bridge without running the full detector stack.

Useful for:
  - Testing MAVLink bridge packet generation
  - Verifying QGroundControl receives STATUSTEXT alerts
  - Integration testing with colcon mock sensors

Usage:
    ros2 run mavlink-bridge test_gps_spoof_alert_generator
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AlertGenerator(Node):
    def __init__(self):
        super().__init__('test_alert_generator')
        self.alert_pub = self.create_publisher(String, '/gps_spoof_alert', 10)
        self.alert_id = 1

        # Send test alerts in sequence
        self.create_timer(2.0, self._generate_alert)
        self._test_sequence = 0

        self.get_logger().info('Test Alert Generator started (emits every 2s)')

    def _generate_alert(self):
        """Generate a test alert."""
        tests = [
            {
                'name': 'INFO - Heading OK',
                'alert': {
                    'alert_id': self.alert_id,
                    'level': 'INFO',
                    'strategy': 'HEADING',
                    'state': 'NOMINAL',
                    'detail': {
                        'ekf2_heading_deg': 45.0,
                        'mag_heading_deg': 45.2,
                        'diff_deg': 0.2,
                        'description': 'EKF2 and magnetometer agree — normal operation'
                    },
                    'timestamp_us': 0
                }
            },
            {
                'name': 'WARNING - Heading Divergence',
                'alert': {
                    'alert_id': self.alert_id + 1,
                    'level': 'WARNING',
                    'strategy': 'HEADING',
                    'state': 'SUSPICIOUS',
                    'detail': {
                        'ekf2_heading_deg': 45.0,
                        'mag_heading_deg': 60.5,
                        'diff_deg': 15.5,
                        'description': 'EKF2 and magnetometer headings diverging'
                    },
                    'timestamp_us': 0
                }
            },
            {
                'name': 'CRITICAL - GPS Spoofing Detected',
                'alert': {
                    'alert_id': self.alert_id + 2,
                    'level': 'CRITICAL',
                    'strategy': 'ALTITUDE',
                    'state': 'SPOOFING_DETECTED',
                    'detail': {
                        'gps_alt_m': 10.5,
                        'baro_alt_m': 100.2,
                        'discrepancy_m': 89.7,
                        'description': 'GPS altitude spoofing detected — barometer disagrees strongly'
                    },
                    'timestamp_us': 0
                }
            },
            {
                'name': 'WARNING - Altitude Divergence',
                'alert': {
                    'alert_id': self.alert_id + 3,
                    'level': 'WARNING',
                    'strategy': 'ALTITUDE',
                    'state': 'SUSPICIOUS',
                    'detail': {
                        'gps_alt_m': 50.0,
                        'baro_alt_m': 45.2,
                        'discrepancy_m': 4.8,
                        'description': 'GPS altitude delta diverging from barometer'
                    },
                    'timestamp_us': 0
                }
            },
            {
                'name': 'CRITICAL - PX4 Hardware Spoof Flag',
                'alert': {
                    'alert_id': self.alert_id + 4,
                    'level': 'CRITICAL',
                    'strategy': 'PX4_INTERNAL',
                    'state': 'SPOOFING_DETECTED',
                    'detail': {
                        'spoofing_state': 2,
                        'jamming_state': 0,
                        'description': 'GPS receiver hardware flag active: spoofing_state=2, jamming_state=0'
                    },
                    'timestamp_us': 0
                }
            },
        ]

        if self._test_sequence < len(tests):
            test = tests[self._test_sequence]
            alert = test['alert']

            msg = String()
            msg.data = json.dumps(alert)

            self.get_logger().info(f"Publishing test alert: {test['name']}")
            self.alert_pub.publish(msg)

            self._test_sequence += 1
        else:
            self.get_logger().info('All test alerts sent; repeating...')
            self._test_sequence = 0


def main(args=None):
    rclpy.init(args=args)
    node = AlertGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
