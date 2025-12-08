#!/usr/bin/env python3
# tf_topics.py - Monitor TF topics and show published transform pairs
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import time
from collections import defaultdict
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage

SAMPLE_DURATION = 5.0


class TfTopicsMonitor(Node):
    """Monitors /tf and /tf_static topics to show published transform pairs."""

    def __init__(self):
        super().__init__('tf_topics_monitor')

        # Store transform pairs and their info
        # Key: (parent_frame, child_frame, topic)
        # Value: {'count': int, 'first_seen': float, 'last_seen': float, 'last_stamp': timestamp}
        self._transforms = {}

        # QoS profile for /tf_static with TRANSIENT_LOCAL durability
        # This allows us to receive the last published static transforms
        static_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribe to both TF topics
        self._tf_sub = self.create_subscription(
            TFMessage, '/tf', self._tf_callback, 10)
        self._tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_callback, static_qos)

    def _tf_callback(self, msg):
        """Handle messages from /tf topic."""
        self._process_transforms(msg, '/tf')

    def _tf_static_callback(self, msg):
        """Handle messages from /tf_static topic."""
        self._process_transforms(msg, '/tf_static')

    def _process_transforms(self, msg, topic):
        """Process transforms from a TF message."""
        current_time = time.time()

        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            key = (parent, child, topic)

            # Convert ROS timestamp to seconds
            stamp_sec = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9

            if key in self._transforms:
                self._transforms[key]['count'] += 1
                self._transforms[key]['last_seen'] = current_time
                self._transforms[key]['last_stamp'] = stamp_sec
            else:
                self._transforms[key] = {
                    'count': 1,
                    'first_seen': current_time,
                    'last_seen': current_time,
                    'last_stamp': stamp_sec
                }

    def monitor_topics(self):
        """Monitor TF topics for a duration and report results."""
        print(f"Monitoring /tf and /tf_static topics for {SAMPLE_DURATION} seconds...")
        print()

        start_time = time.time()
        while time.time() - start_time < SAMPLE_DURATION:
            rclpy.spin_once(self, timeout_sec=0.1)

        self._print_results()

    def _format_timestamp(self, stamp_sec):
        """Format a timestamp as a readable date/time string."""
        if stamp_sec == 0:
            return "0 (not set)"
        dt = datetime.fromtimestamp(stamp_sec)
        return dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    def _format_age(self, seconds):
        """Format age in seconds as hh:mm:ss.ms."""
        abs_seconds = abs(seconds)
        hours = int(abs_seconds // 3600)
        minutes = int((abs_seconds % 3600) // 60)
        secs = abs_seconds % 60
        sign = "-" if seconds < 0 else ""
        return f"{sign}{hours:02d}:{minutes:02d}:{secs:06.3f}"

    def _print_results(self):
        """Print the collected transform pairs grouped by topic."""
        if not self._transforms:
            print("No TF transforms detected.")
            return

        current_time = time.time()

        # Group by topic
        tf_transforms = []
        tf_static_transforms = []

        for (parent, child, topic), info in self._transforms.items():
            if topic == '/tf':
                tf_transforms.append((parent, child, info))
            else:
                tf_static_transforms.append((parent, child, info))

        # Print /tf transforms
        if tf_transforms:
            print("=" * 80)
            print("/tf Topic (Dynamic Transforms)")
            print("=" * 80)
            print()

            for parent, child, info in sorted(tf_transforms):
                print(f"Transform: {parent} → {child}")
                print(f"  Messages received: {info['count']}")

                # Calculate rate
                duration = info['last_seen'] - info['first_seen']
                if duration > 0:
                    rate = info['count'] / duration
                    print(f"  Rate: {rate:.2f} Hz")

                # Show last timestamp
                age = current_time - info['last_stamp']
                age_str = self._format_age(age)
                print(f"  Last transform timestamp: {self._format_timestamp(info['last_stamp'])}")
                print(f"  Age: {age_str} ago")
                print()

        # Print /tf_static transforms
        if tf_static_transforms:
            print("=" * 80)
            print("/tf_static Topic (Static Transforms)")
            print("=" * 80)
            print()

            for parent, child, info in sorted(tf_static_transforms):
                print(f"Transform: {parent} → {child}")
                print(f"  Messages received: {info['count']}")

                # Show last timestamp and age
                age = current_time - info['last_stamp']
                age_str = self._format_age(age)
                print(f"  Last transform timestamp: {self._format_timestamp(info['last_stamp'])}")
                print(f"  Age: {age_str} ago")
                print()

        # Print summary
        print("=" * 80)
        print("Summary")
        print("=" * 80)
        print(f"Total transform pairs detected: {len(self._transforms)}")
        print(f"  /tf pairs: {len(tf_transforms)}")
        print(f"  /tf_static pairs: {len(tf_static_transforms)}")


def monitor_tf_topics():
    """Monitor TF topics and show published transform pairs."""
    rclpy.init()
    monitor = TfTopicsMonitor()
    monitor.monitor_topics()
    monitor.destroy_node()
    rclpy.shutdown()


def main(args=None):
    """Main entry point for CLI."""
    import sys

    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        print("Usage: tf_topics")
        print("\nMonitor /tf and /tf_static topics and show published transform pairs")
        print("\nThis tool monitors both TF topics and reports:")
        print("  - Transform pairs being published")
        print("  - Publishing rates for dynamic transforms")
        print("  - Timestamps and ages of transforms")
        sys.exit(0)

    monitor_tf_topics()


if __name__ == "__main__":
    main()
