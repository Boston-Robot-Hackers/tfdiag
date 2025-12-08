#!/usr/bin/env python3
# filepath: /home/pitosalas/ros2_ws/src/tfdiag/tfdiag/time_checker.py
# time_checker.py - Checks message timestamps against ROS2 and system time
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

SAMPLE_DURATION = 5.0
TIME_DIFF_WARNING_MS = 25.0
TIME_DIFF_ERROR_MS = 100.0

# Topics to check for timestamp synchronization
# Format: (topic_name, message_type_string)
TOPICS_TO_CHECK = [
    ("/odom", "nav_msgs/msg/Odometry"),
    ("/odom/unfiltered", "nav_msgs/msg/Odometry"),
    ("/imu", "sensor_msgs/msg/Imu"),
    ("/imu/data", "sensor_msgs/msg/Imu"),
    ("/tf", "tf2_msgs/msg/TFMessage"),
    ("/tf_static", "tf2_msgs/msg/TFMessage"),
    # Add more topics as needed
]


class TimeChecker(Node):
    """Checks message timestamps against ROS2 and system time."""

    def __init__(self):
        super().__init__("time_checker")
        self._topic_data = {}

    def subscribe_to_topic(self, topic_name, msg_type):
        # QoS depth=10 buffers last 10 messages to avoid missing data during processing
        qos_profile = QoSProfile(depth=10)

        def callback(msg):
            ros_now = self.get_clock().now()
            system_now = time.time()

            # Get timestamp based on message structure
            timestamp = self._get_timestamp(msg)

            # Extract sec and nanosec from the timestamp
            msg_stamp_sec = timestamp.sec
            msg_stamp_nanosec = timestamp.nanosec
            msg_stamp_total = msg_stamp_sec + msg_stamp_nanosec / 1e9

            ros_now_sec = (
                ros_now.seconds_nanoseconds()[0]
                + ros_now.seconds_nanoseconds()[1] / 1e9
            )
            ros_diff = (ros_now_sec - msg_stamp_total) * 1000.0
            system_diff = (system_now - msg_stamp_total) * 1000.0

            # setdefault creates empty list if topic_name not in dict, then appends to it
            self._topic_data.setdefault(topic_name, []).append(
                {
                    "msg_time": msg_stamp_total,
                    "ros_time": ros_now_sec,
                    "system_time": system_now,
                    "ros_diff_ms": ros_diff,
                    "system_diff_ms": system_diff,
                }
            )

        self.create_subscription(msg_type, topic_name, callback, qos_profile)

    def _check_tf_timestamps(self, msg):
        if not hasattr(msg, "transforms") or len(msg.transforms) <= 1:
            return

        timestamps = [
            t.header.stamp.sec + t.header.stamp.nanosec / 1e9
            for t in msg.transforms
        ]
        min_ts = min(timestamps)
        max_ts = max(timestamps)
        diff_ms = (max_ts - min_ts) * 1000.0

        if diff_ms > 1.0:
            self.get_logger().error(
                f"TF timestamp mismatch: {len(timestamps)} transforms differ by {diff_ms:.2f} ms"
            )

    def _get_timestamp(self, msg):
        # Most messages have header.stamp (Odometry, Imu, LaserScan, etc.)
        if hasattr(msg, "header"):
            return msg.header.stamp

        # TF messages have an array of transforms
        if hasattr(msg, "transforms"):
            self._check_tf_timestamps(msg)
            return msg.transforms[0].header.stamp

        # Some messages have a direct stamp field
        return msg.stamp

    def sample_topics(self):
        print(f"Sampling timestamps for {SAMPLE_DURATION} seconds...")
        start_time = time.time()
        while time.time() - start_time < SAMPLE_DURATION:
            rclpy.spin_once(self, timeout_sec=0.1)

    def analyze_results(self):
        print(f"\nTimestamp Analysis for {len(self._topic_data)} topics:\n")

        for topic_name, samples in sorted(self._topic_data.items()):
            print(f"Topic: {topic_name}")
            print(f"  Samples collected: {len(samples)}")

            ros_diffs = [s["ros_diff_ms"] for s in samples]
            system_diffs = [s["system_diff_ms"] for s in samples]

            avg_ros_diff = sum(ros_diffs) / len(ros_diffs)
            max_ros_diff = max(ros_diffs)
            min_ros_diff = min(ros_diffs)

            avg_system_diff = sum(system_diffs) / len(system_diffs)
            max_system_diff = max(system_diffs)
            min_system_diff = min(system_diffs)

            print("  ROS time difference:")
            print(f"    Average: {avg_ros_diff:.2f} ms")
            print(f"    Range: [{min_ros_diff:.2f}, {max_ros_diff:.2f}] ms")

            print("  System time difference:")
            print(f"    Average: {avg_system_diff:.2f} ms")
            print(f"    Range: [{min_system_diff:.2f}, {max_system_diff:.2f}] ms")

            if abs(avg_ros_diff) > TIME_DIFF_ERROR_MS:
                print(
                    f"  ❌ ERROR: Average ROS time diff exceeds {TIME_DIFF_ERROR_MS} ms"
                )
            elif abs(avg_ros_diff) > TIME_DIFF_WARNING_MS:
                print(
                    f"  ⚠️  WARNING: Average ROS time diff exceeds {TIME_DIFF_WARNING_MS} ms"
                )
            else:
                print("  ✅ OK: Timestamps are synchronized")

            print()

    def load_topics_from_config(self, topic_configs):
        # Returns list of (topic_name, msg_class) tuples
        available_topics = []

        for topic_name, type_string in topic_configs:
            # Parse message type string (e.g., 'nav_msgs/msg/Odometry')
            pkg, _, msg = type_string.split("/")
            module = __import__(f"{pkg}.msg", fromlist=[msg])
            msg_class = getattr(module, msg)
            available_topics.append((topic_name, msg_class))

        return available_topics


def check_timestamps(topic_configs=None):
    rclpy.init()
    checker = TimeChecker()

    # Use default config if none provided
    if topic_configs is None:
        topic_configs = TOPICS_TO_CHECK

    print(f"Checking timestamps for {len(topic_configs)} configured topics...")
    print("Waiting for topic discovery...")
    time.sleep(2.0)  # Allow time for DDS discovery

    topics = checker.load_topics_from_config(topic_configs)

    print(f"\nSubscribing to {len(topics)} topics...")
    for topic_name, msg_type in topics:
        checker.subscribe_to_topic(topic_name, msg_type)

    checker.sample_topics()
    checker.analyze_results()

    checker.destroy_node()
    rclpy.shutdown()


def main(args=None):
    """Main entry point for CLI."""
    import sys

    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        print("Usage: tf_timechecker")
        print("\nCheck message timestamps against ROS2 and system time")
        print("\nThis tool monitors configured topics and checks:")
        print("  - Message timestamp vs ROS2 time")
        print("  - Message timestamp vs system time")
        print("  - Synchronization quality")
        print("\nTopics checked:")
        for topic, msg_type in TOPICS_TO_CHECK:
            print(f"  {topic} ({msg_type})")
        sys.exit(0)

    check_timestamps()


if __name__ == "__main__":
    main()
