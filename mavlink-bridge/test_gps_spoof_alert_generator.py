#!/usr/bin/env python3
"""
Manual QGC test tool: publishes synthetic GPS-spoofing alerts to
/gps_spoof_alert, standing in for gps_spoof_detector_node so the
detector -> gps_spoof_mavlink_bridge -> QGroundControl pipeline can be
exercised without triggering a real spoofing condition.

Referenced by docs/INTEGRATION_TEST_CHECKLIST.md's Phase 5.2 (basic
reception), 5.4 (stress test), and 7.1 (high-frequency stream) -- this
script replaces the inline Python snippets those phases used while this
file didn't exist.

Usage:
  ros2 run mavlink-bridge test_gps_spoof_alert_generator
  ros2 run mavlink-bridge test_gps_spoof_alert_generator --count 100 --rate 100 --level CRITICAL
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LEVELS = ['INFO', 'WARNING', 'CRITICAL']
STATE_FOR_LEVEL = {
    'INFO': 'NOMINAL',
    'WARNING': 'SUSPICIOUS',
    'CRITICAL': 'SPOOFING_DETECTED',
}


class AlertGenerator(Node):
    def __init__(self, topic: str):
        super().__init__('test_gps_spoof_alert_generator')
        self._pub = self.create_publisher(String, topic, 10)

    def publish_alert(self, alert_id: int, level: str, strategy: str, description: str):
        msg = String()
        msg.data = json.dumps({
            'alert_id': alert_id,
            'level': level,
            'strategy': strategy,
            'state': STATE_FOR_LEVEL[level],
            'detail': {'description': description},
            'timestamp_us': int(time.time() * 1_000_000),
        })
        self._pub.publish(msg)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, default=3,
                         help='Number of alerts to publish (default: 3, one per severity level)')
    parser.add_argument('--rate', type=float, default=1.0,
                         help='Alerts per second (default: 1.0)')
    parser.add_argument('--level', choices=LEVELS, default=None,
                         help='Fixed severity for every alert (default: cycle INFO/WARNING/CRITICAL)')
    parser.add_argument('--strategy', default='HEADING',
                         help="Detection strategy label to report (default: 'HEADING')")
    parser.add_argument('--topic', default='/gps_spoof_alert',
                         help="Topic to publish on (default: '/gps_spoof_alert')")
    return parser.parse_args()


def main():
    ns = parse_args()
    rclpy.init()
    node = AlertGenerator(ns.topic)

    try:
        for i in range(ns.count):
            level = ns.level or LEVELS[i % len(LEVELS)]
            description = f'{level} test alert #{i} ({ns.strategy})'
            node.publish_alert(i, level, ns.strategy, description)
            node.get_logger().info(f'Published {level} alert #{i}: {description!r}')
            rclpy.spin_once(node, timeout_sec=0.0)
            if i < ns.count - 1:
                time.sleep(1.0 / ns.rate)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
