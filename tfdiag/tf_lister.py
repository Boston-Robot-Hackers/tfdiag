#!/usr/bin/env python3
# tf_lister.py - Lists all TF frames and their information
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import time
from datetime import datetime

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_msgs.msg import TFMessage
import yaml

SAMPLE_DURATION = 5.0


class TfLister(Node):
    """Lists all TF frames in the system."""

    def __init__(self):
        super().__init__('tf_lister')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._broadcaster_nodes = set()
        self._frame_to_node = {}
        self._frame_to_node_map = {}

        self._tf_sub = self.create_subscription(
            TFMessage, '/tf', self._tf_callback, 10)
        self._tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_callback, 10)

    def _tf_callback(self, msg):
        """Track which TF frames are published on /tf topic."""
        for transform in msg.transforms:
            child_frame = transform.child_frame_id
            if child_frame not in self._frame_to_node:
                self._frame_to_node[child_frame] = '/tf'

    def _tf_static_callback(self, msg):
        """Track which TF frames are published on /tf_static topic."""
        for transform in msg.transforms:
            child_frame = transform.child_frame_id
            if child_frame not in self._frame_to_node:
                self._frame_to_node[child_frame] = '/tf_static'

    def get_broadcaster_nodes(self):
        """Get actual node names broadcasting TF frames."""
        tf_pubs = self.get_publishers_info_by_topic('/tf')
        tf_static_pubs = self.get_publishers_info_by_topic('/tf_static')

        broadcaster_nodes = set()
        tf_nodes = {}
        tf_static_nodes = {}

        for info in tf_pubs:
            full_name = f'{info.node_namespace}{info.node_name}'
            broadcaster_nodes.add(full_name)
            tf_nodes[full_name] = info

        for info in tf_static_pubs:
            full_name = f'{info.node_namespace}{info.node_name}'
            broadcaster_nodes.add(full_name)
            tf_static_nodes[full_name] = info

        return broadcaster_nodes, tf_nodes, tf_static_nodes

    def map_frames_to_nodes(self, tf_nodes, tf_static_nodes, all_frames):
        """Map each frame to publisher nodes."""
        frame_node_map = {}

        for frame in all_frames:
            if frame in self._frame_to_node:
                topic = self._frame_to_node[frame]
                frame_node_map[frame] = []
                if topic == '/tf':
                    for node_name in tf_nodes.keys():
                        frame_node_map[frame].append(node_name)
                elif topic == '/tf_static':
                    for node_name in tf_static_nodes.keys():
                        frame_node_map[frame].append(node_name)
            else:
                frame_node_map[frame] = list(tf_static_nodes.keys())

        return frame_node_map

    def get_all_frames_info(self):
        """Get information about all TF frames."""
        print(f'Sampling TFs for {SAMPLE_DURATION} seconds...')
        start_time = time.time()
        while time.time() - start_time < SAMPLE_DURATION:
            rclpy.spin_once(self, timeout_sec=0.1)

        frames_yaml = self._tf_buffer.all_frames_as_yaml()
        frames_dict = yaml.safe_load(frames_yaml)

        if not frames_dict:
            return {}

        result = self.get_broadcaster_nodes()
        self._broadcaster_nodes = result[0]
        tf_nodes = result[1]
        tf_static_nodes = result[2]

        all_frames = list(frames_dict.keys())

        self._frame_to_node_map = self.map_frames_to_nodes(
            tf_nodes, tf_static_nodes, all_frames)

        return frames_dict

    def _format_age(self, seconds):
        """Format age in seconds as hh:mm:ss.ms."""
        abs_seconds = abs(seconds)
        hours = int(abs_seconds // 3600)
        minutes = int((abs_seconds % 3600) // 60)
        secs = abs_seconds % 60
        sign = "-" if seconds < 0 else ""
        return f"{sign}{hours:02d}:{minutes:02d}:{secs:06.3f}"

    def print_frames_info(self, include_static):
        """Print information about all TF frames."""
        frames_dict = self.get_all_frames_info()

        if not frames_dict:
            print('No TF frames found')
            return

        current_time = time.time()

        print(f'\nBroadcaster Nodes: {", ".join(sorted(self._broadcaster_nodes))}')
        print(f'\nFound {len(frames_dict)} TF frames:\n')

        for frame_name, frame_info in sorted(frames_dict.items()):
            is_static = frame_info.get('rate', 0) >= 10000.0

            if is_static and not include_static:
                continue

            static_label = ' (static frame)' if is_static else ''

            print(f'Frame: {frame_name}{static_label}')

            if 'parent' in frame_info:
                print(f'  Parent: {frame_info["parent"]}')

            if frame_name in self._frame_to_node_map:
                nodes = ', '.join(sorted(self._frame_to_node_map[frame_name]))
                print(f'  Published by: {nodes}')

            rate_str = ""
            buffer_str = ""
            if 'rate' in frame_info:
                rate_str = f'Rate: {frame_info["rate"]:.2f} Hz'
            if 'buffer_length' in frame_info:
                buffer_str = f'Buffer: {frame_info["buffer_length"]:.2f}s'
            if rate_str or buffer_str:
                parts = [p for p in [rate_str, buffer_str] if p]
                print(f'  {", ".join(parts)}')

            last_str = ""
            oldest_str = ""
            if "most_recent_transform" in frame_info:
                timestamp = frame_info["most_recent_transform"]
                age = current_time - timestamp
                age_str = self._format_age(age)
                last_str = f"Last: {age_str} ago"
            if "oldest_transform" in frame_info:
                timestamp = frame_info["oldest_transform"]
                age = current_time - timestamp
                age_str = self._format_age(age)
                oldest_str = f"Oldest: {age_str} ago"
            if last_str or oldest_str:
                parts = [p for p in [last_str, oldest_str] if p]
                print(f'  {", ".join(parts)}')

            print()


def list_tf_frames(include_static):
    """List TF frames."""
    rclpy.init()
    lister = TfLister()
    lister.print_frames_info(include_static)
    lister.destroy_node()
    rclpy.shutdown()
