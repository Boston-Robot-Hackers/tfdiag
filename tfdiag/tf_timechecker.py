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
TIME_DIFF_WARNING_MS = 100.0
TIME_DIFF_ERROR_MS = 1000.0

# Topics to check for timestamp synchronization
# Format: (topic_name, message_type_string)
TOPICS_TO_CHECK = [
    ('/odom', 'nav_msgs/msg/Odometry'),
    ('/odom/unfiltered', 'nav_msgs/msg/Odometry'),
    ('/imu', 'sensor_msgs/msg/Imu'),
    ('/imu/data', 'sensor_msgs/msg/Imu'),
    ('/tf', 'tf2_msgs/msg/TFMessage'),
    ('/tf_static', 'tf2_msgs/msg/TFMessage'),
    # Add more topics as needed
]


class TimeChecker(Node):
    """Checks message timestamps against ROS2 and system time."""

    def __init__(self):
        super().__init__('time_checker')
        self._topic_data = {}

    def subscribe_to_topic(self, topic_name, msg_type):
        """Subscribe to a topic and track its timestamps.

        Args:
            topic_name: Name of the topic to subscribe to
            msg_type: Message class type
        """
        qos_profile = QoSProfile(depth=10)

        def callback(msg):
            ros_now = self.get_clock().now()
            system_now = time.time()

            try:
                # Get timestamp based on message structure
                timestamp = self._get_timestamp(msg)

                if timestamp is None:
                    return

                # Extract sec and nanosec from the timestamp
                msg_stamp_sec = timestamp.sec
                msg_stamp_nanosec = timestamp.nanosec
                msg_stamp_total = msg_stamp_sec + msg_stamp_nanosec / 1e9

                ros_now_sec = ros_now.seconds_nanoseconds()[0] + ros_now.seconds_nanoseconds()[1] / 1e9
                ros_diff = (ros_now_sec - msg_stamp_total) * 1000.0
                system_diff = (system_now - msg_stamp_total) * 1000.0

                if topic_name not in self._topic_data:
                    self._topic_data[topic_name] = []

                self._topic_data[topic_name].append({
                    'msg_time': msg_stamp_total,
                    'ros_time': ros_now_sec,
                    'system_time': system_now,
                    'ros_diff_ms': ros_diff,
                    'system_diff_ms': system_diff
                })
            except (AttributeError, IndexError) as e:
                self.get_logger().warning(f"Failed to extract timestamp from {topic_name}: {e}")

        self.create_subscription(msg_type, topic_name, callback, qos_profile)

    def _get_timestamp(self, msg):
        """Get timestamp from a message based on common ROS2 patterns.

        Args:
            msg: The ROS2 message object

        Returns:
            The timestamp object, or None if not found
        """
        # Most messages have header.stamp (Odometry, Imu, LaserScan, etc.)
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            return msg.header.stamp

        # TF messages have an array of transforms
        if hasattr(msg, 'transforms') and len(msg.transforms) > 0:
            return msg.transforms[0].header.stamp

        # Some messages have a direct stamp field
        if hasattr(msg, 'stamp'):
            return msg.stamp

        # Could not find a timestamp
        return None

    def sample_topics(self):
        """Sample topics for the configured duration."""
        print(f'Sampling timestamps for {SAMPLE_DURATION} seconds...')
        start_time = time.time()
        while time.time() - start_time < SAMPLE_DURATION:
            rclpy.spin_once(self, timeout_sec=0.1)

    def analyze_results(self):
        """Analyze collected timestamp data."""
        if not self._topic_data:
            print('No timestamp data collected')
            return

        print(f'\nTimestamp Analysis for {len(self._topic_data)} topics:\n')

        for topic_name, samples in sorted(self._topic_data.items()):
            if not samples:
                continue

            print(f'Topic: {topic_name}')
            print(f'  Samples collected: {len(samples)}')

            ros_diffs = [s['ros_diff_ms'] for s in samples]
            system_diffs = [s['system_diff_ms'] for s in samples]

            avg_ros_diff = sum(ros_diffs) / len(ros_diffs)
            max_ros_diff = max(ros_diffs)
            min_ros_diff = min(ros_diffs)

            avg_system_diff = sum(system_diffs) / len(system_diffs)
            max_system_diff = max(system_diffs)
            min_system_diff = min(system_diffs)

            print(f'  ROS time difference:')
            print(f'    Average: {avg_ros_diff:.2f} ms')
            print(f'    Range: [{min_ros_diff:.2f}, {max_ros_diff:.2f}] ms')

            print(f'  System time difference:')
            print(f'    Average: {avg_system_diff:.2f} ms')
            print(f'    Range: [{min_system_diff:.2f}, {max_system_diff:.2f}] ms')

            if abs(avg_ros_diff) > TIME_DIFF_ERROR_MS:
                print(f'  ❌ ERROR: Average ROS time diff exceeds {TIME_DIFF_ERROR_MS} ms')
            elif abs(avg_ros_diff) > TIME_DIFF_WARNING_MS:
                print(f'  ⚠️  WARNING: Average ROS time diff exceeds {TIME_DIFF_WARNING_MS} ms')
            else:
                print(f'  ✅ OK: Timestamps are synchronized')

            print()

    def load_topics_from_config(self, topic_configs):
        """Load message types from topic configuration.

        Args:
            topic_configs: List of (topic_name, type_string) tuples

        Returns:
            List of (topic_name, msg_class) tuples that are available
        """
        available_topics = []
        current_topics = {name for name, _ in self.get_topic_names_and_types()}

        for config in topic_configs:
            if len(config) != 2:
                print(f"Warning: Invalid config format {config}, expected (topic, type)")
                continue

            topic_name, type_string = config

            # Check if topic exists
            if topic_name not in current_topics:
                print(f"Warning: Topic '{topic_name}' not found, skipping")
                continue

            try:
                # Parse message type string (e.g., 'nav_msgs/msg/Odometry')
                parts = type_string.split('/')
                if len(parts) != 3:
                    print(f"Warning: Invalid type string '{type_string}', skipping {topic_name}")
                    continue

                pkg, _, msg = parts
                module = __import__(f'{pkg}.msg', fromlist=[msg])
                msg_class = getattr(module, msg)

                available_topics.append((topic_name, msg_class))
                print(f"Loaded: {topic_name} ({type_string})")

            except Exception as e:
                print(f"Warning: Failed to load {topic_name} ({type_string}): {e}")
                continue

        return available_topics


def check_timestamps(topic_configs=None):
    """Check timestamps on specified topics.

    Args:
        topic_configs: List of (topic_name, type_string) tuples.
                      If None, uses TOPICS_TO_CHECK.
                      Example: [('/odom', 'nav_msgs/msg/Odometry')]
    """
    rclpy.init()
    checker = TimeChecker()

    # Use default config if none provided
    if topic_configs is None:
        topic_configs = TOPICS_TO_CHECK

    print(f'Checking timestamps for {len(topic_configs)} configured topics...')
    print('Waiting for topic discovery...')
    time.sleep(2.0)  # Allow time for DDS discovery

    # Load available topics from config
    topics = checker.load_topics_from_config(topic_configs)

    if not topics:
        print('No topics available to check')
        checker.destroy_node()
        rclpy.shutdown()
        return

    print(f'\nSubscribing to {len(topics)} topics...')
    for topic_name, msg_type in topics:
        checker.subscribe_to_topic(topic_name, msg_type)

    checker.sample_topics()
    checker.analyze_results()

    checker.destroy_node()
    rclpy.shutdown()