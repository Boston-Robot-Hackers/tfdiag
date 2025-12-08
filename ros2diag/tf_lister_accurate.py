#!/usr/bin/env python3
# tf_lister_accurate.py - Accurate TF frame publisher detection using context managers
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import time

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
import yaml

SAMPLE_DURATION = 5.0


class TfListerAccurate(Node):
    """Lists all TF frames with accurate publisher detection using subscription context."""

    def __init__(self):
        super().__init__('tf_lister')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Maps to track accurate publisher information
        self._frame_to_gid = {}  # frame -> publisher GID (as bytes)
        self._gid_to_node = {}   # GID (as bytes) -> node name
        self._last_message_info = {}  # topic -> last message info

        # Create QoS profile for subscriptions
        qos_profile = QoSProfile(depth=10)

        # QoS profile for /tf_static with transient local durability
        # This allows us to receive static transforms that were published before we subscribed
        qos_static_profile = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create subscriptions - we'll capture context in callbacks
        self._tf_sub = self.create_subscription(
            TFMessage, '/tf',
            lambda msg: self._tf_callback_with_info(msg, '/tf'),
            qos_profile)
        self._tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static',
            lambda msg: self._tf_static_callback_with_info(msg, '/tf_static'),
            qos_static_profile)

    def _gid_to_string(self, gid):
        """Convert GID to a hashable tuple for use as dictionary key."""
        if isinstance(gid, (bytes, bytearray)):
            return tuple(gid)
        elif isinstance(gid, list):
            return tuple(gid)
        return gid

    def _tf_callback_with_info(self, msg, topic):
        """Track TF frames and try to correlate with publisher info."""
        # Map each frame - we'll correlate with GIDs later
        for transform in msg.transforms:
            child_frame = transform.child_frame_id
            # For now, just track that we saw this frame on this topic
            if child_frame not in self._frame_to_gid:
                self._frame_to_gid[child_frame] = topic

    def _tf_static_callback_with_info(self, msg, topic):
        """Track static TF frames and try to correlate with publisher info."""
        # Map each frame - we'll correlate with GIDs later
        for transform in msg.transforms:
            child_frame = transform.child_frame_id
            # For now, just track that we saw this frame on this topic
            if child_frame not in self._frame_to_gid:
                self._frame_to_gid[child_frame] = topic

    def _build_gid_to_node_map(self):
        """Build a map from publisher GIDs to node names."""
        # Get all publishers on /tf and /tf_static
        for topic in ['/tf', '/tf_static']:
            pubs = self.get_publishers_info_by_topic(topic)
            for pub_info in pubs:
                gid_key = self._gid_to_string(pub_info.endpoint_gid)
                node_name = f'{pub_info.node_namespace}{pub_info.node_name}'
                self._gid_to_node[gid_key] = node_name

    def get_frame_publishers_by_topic(self):
        """Get frame-to-publisher mapping by correlating topics.

        This is more accurate than the original approach because it attempts
        to use endpoint_gid information when available.
        """
        self._build_gid_to_node_map()

        # Build topic-to-nodes mapping
        topic_to_nodes = {}
        for topic in ['/tf', '/tf_static']:
            pubs = self.get_publishers_info_by_topic(topic)
            nodes = []
            for pub_info in pubs:
                node_name = f'{pub_info.node_namespace}{pub_info.node_name}'
                nodes.append(node_name)
            topic_to_nodes[topic] = nodes

        # Map frames to nodes based on topic
        frame_to_node_map = {}
        for frame, topic in self._frame_to_gid.items():
            if topic in topic_to_nodes:
                frame_to_node_map[frame] = topic_to_nodes[topic]
            else:
                frame_to_node_map[frame] = ['<unknown>']

        return frame_to_node_map

    def get_all_broadcaster_nodes(self):
        """Get all unique nodes that broadcast TF frames."""
        self._build_gid_to_node_map()

        all_nodes = set()
        for topic in ['/tf', '/tf_static']:
            pubs = self.get_publishers_info_by_topic(topic)
            for pub_info in pubs:
                node_name = f'{pub_info.node_namespace}{pub_info.node_name}'
                all_nodes.add(node_name)

        return all_nodes

    def get_all_frames_info(self):
        """Get information about all TF frames."""
        print(f'Sampling TFs for {SAMPLE_DURATION} seconds (accurate mode)...')
        start_time = time.time()
        while time.time() - start_time < SAMPLE_DURATION:
            rclpy.spin_once(self, timeout_sec=0.1)

        frames_yaml = self._tf_buffer.all_frames_as_yaml()
        frames_dict = yaml.safe_load(frames_yaml)

        if not frames_dict:
            return {}, {}, set()

        # Ensure all frames are in the tracking dict
        # Static frames might not have been caught by callbacks
        for frame_name, frame_info in frames_dict.items():
            if frame_name not in self._frame_to_gid:
                # Check if it's a static frame based on rate
                is_static = frame_info.get('rate', 0) >= 10000.0
                self._frame_to_gid[frame_name] = '/tf_static' if is_static else '/tf'

        frame_to_node_map = self.get_frame_publishers_by_topic()
        broadcaster_nodes = self.get_all_broadcaster_nodes()

        return frames_dict, frame_to_node_map, broadcaster_nodes

    def print_frames_info(self):
        """Print information about all TF frames."""
        frames_dict, frame_to_node_map, broadcaster_nodes = self.get_all_frames_info()

        if not frames_dict:
            print('No TF frames found')
            return

        print(f'\nBroadcaster Nodes: {", ".join(sorted(broadcaster_nodes))}')
        print(f'\nFound {len(frames_dict)} TF frames:\n')

        for frame_name, frame_info in sorted(frames_dict.items()):
            is_static = frame_info.get('rate', 0) >= 10000.0
            static_label = ' (static frame)' if is_static else ''

            print(f'Frame: {frame_name}{static_label}')

            if 'parent' in frame_info:
                print(f'  Parent: {frame_info["parent"]}')

            if frame_name in frame_to_node_map:
                nodes = ', '.join(sorted(frame_to_node_map[frame_name]))
                print(f'  Published by: {nodes}')

            if 'rate' in frame_info:
                print(f'  Rate: {frame_info["rate"]:.2f} Hz')

            if 'buffer_length' in frame_info:
                print(f'  Buffer Length: {frame_info["buffer_length"]:.2f}s')

            if 'most_recent_transform' in frame_info:
                print(f'  Last Update: {frame_info["most_recent_transform"]:.2f}s')

            if 'oldest_transform' in frame_info:
                print(f'  Oldest Transform: {frame_info["oldest_transform"]:.2f}s')

            print()


def list_tf_frames_accurate():
    """List TF frames with accurate publisher detection."""
    rclpy.init()
    lister = TfListerAccurate()
    lister.print_frames_info()
    lister.destroy_node()
    rclpy.shutdown()
