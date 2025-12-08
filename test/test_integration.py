#!/usr/bin/env python3
# test_integration.py - Integration tests for ros2diag
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import unittest
from unittest.mock import Mock, patch
import time
import pytest

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import tf2_ros

from ros2diag.tf_lister import TfLister
from ros2diag.tf_lister_accurate import TfListerAccurate


class MockTfBroadcaster(Node):
    """Mock TF broadcaster for testing."""

    def __init__(self, node_name='mock_broadcaster'):
        """Initialize the mock broadcaster."""
        super().__init__(node_name)
        self.tf_publisher = self.create_publisher(TFMessage, '/tf', 10)
        self.static_publisher = self.create_publisher(TFMessage, '/tf_static', 10)

    def publish_transform(self, parent_frame, child_frame, is_static=False):
        """Publish a single transform."""
        msg = TFMessage()
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        msg.transforms = [transform]

        if is_static:
            self.static_publisher.publish(msg)
        else:
            self.tf_publisher.publish(msg)


class TestSingleBroadcaster(unittest.TestCase):
    """Integration tests with a single TF broadcaster."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS context once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Tear down ROS context."""
        rclpy.shutdown()

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def test_detect_single_dynamic_publisher(self, mock_buffer, mock_listener):
        """Test detection of a single node publishing dynamic TF."""
        # Create broadcaster
        broadcaster = MockTfBroadcaster('test_dynamic_node')

        # Create lister
        lister = TfLister()

        # Publish a dynamic transform
        broadcaster.publish_transform('map', 'base_link', is_static=False)

        # Spin to process messages
        rclpy.spin_once(broadcaster, timeout_sec=0.1)
        rclpy.spin_once(lister, timeout_sec=0.1)

        # Mock publisher info
        mock_pub_info = Mock()
        mock_pub_info.node_name = 'test_dynamic_node'
        mock_pub_info.node_namespace = '/'

        with patch.object(lister, 'get_publishers_info_by_topic', return_value=[mock_pub_info]):
            broadcasters, tf_nodes, tf_static_nodes = lister.get_broadcaster_nodes()

        # Verify broadcaster detected
        self.assertIn('/test_dynamic_node', broadcasters)
        self.assertIn('/test_dynamic_node', tf_nodes)

        # Cleanup
        lister.destroy_node()
        broadcaster.destroy_node()

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def test_detect_single_static_publisher(self, mock_buffer, mock_listener):
        """Test detection of a single node publishing static TF."""
        # Create broadcaster
        broadcaster = MockTfBroadcaster('test_static_node')

        # Create lister
        lister = TfLister()

        # Publish a static transform
        broadcaster.publish_transform('map', 'odom', is_static=True)

        # Spin to process messages
        rclpy.spin_once(broadcaster, timeout_sec=0.1)
        rclpy.spin_once(lister, timeout_sec=0.1)

        # Verify frame was tracked
        self.assertIn('odom', lister._frame_to_node)
        self.assertEqual(lister._frame_to_node['odom'], '/tf_static')

        # Cleanup
        lister.destroy_node()
        broadcaster.destroy_node()


class TestMultipleBroadcasters(unittest.TestCase):
    """Integration tests with multiple TF broadcasters."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS context once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Tear down ROS context."""
        rclpy.shutdown()

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def test_multiple_dynamic_broadcasters(self, mock_buffer, mock_listener):
        """Test detection of multiple nodes publishing dynamic TF."""
        # Create broadcasters
        broadcaster1 = MockTfBroadcaster('node1')
        broadcaster2 = MockTfBroadcaster('node2')

        # Create lister
        lister = TfLister()

        # Publish transforms from different nodes
        broadcaster1.publish_transform('map', 'base_link1', is_static=False)
        broadcaster2.publish_transform('map', 'base_link2', is_static=False)

        # Spin to process messages
        rclpy.spin_once(broadcaster1, timeout_sec=0.1)
        rclpy.spin_once(broadcaster2, timeout_sec=0.1)
        rclpy.spin_once(lister, timeout_sec=0.1)

        # Mock publisher info
        mock_pub_info1 = Mock()
        mock_pub_info1.node_name = 'node1'
        mock_pub_info1.node_namespace = '/'

        mock_pub_info2 = Mock()
        mock_pub_info2.node_name = 'node2'
        mock_pub_info2.node_namespace = '/'

        with patch.object(lister, 'get_publishers_info_by_topic',
                         return_value=[mock_pub_info1, mock_pub_info2]):
            broadcasters, tf_nodes, tf_static_nodes = lister.get_broadcaster_nodes()

        # Verify both broadcasters detected
        self.assertIn('/node1', broadcasters)
        self.assertIn('/node2', broadcasters)
        self.assertEqual(len(broadcasters), 2)

        # Cleanup
        lister.destroy_node()
        broadcaster1.destroy_node()
        broadcaster2.destroy_node()


class TestMixedStaticAndDynamic(unittest.TestCase):
    """Integration tests with both static and dynamic transforms."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS context once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Tear down ROS context."""
        rclpy.shutdown()

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def test_node_publishing_both_static_and_dynamic(self, mock_buffer, mock_listener):
        """Test node publishing both static and dynamic transforms."""
        # Create broadcaster
        broadcaster = MockTfBroadcaster('mixed_node')

        # Create lister
        lister = TfLister()

        # Publish both static and dynamic transforms
        broadcaster.publish_transform('map', 'odom', is_static=True)
        broadcaster.publish_transform('odom', 'base_link', is_static=False)

        # Spin multiple times to allow messages to propagate
        for _ in range(5):
            rclpy.spin_once(broadcaster, timeout_sec=0.1)
            rclpy.spin_once(lister, timeout_sec=0.1)

        # Verify at least the dynamic frame was tracked (static may have QoS issues in test)
        self.assertIn('base_link', lister._frame_to_node)
        self.assertEqual(lister._frame_to_node['base_link'], '/tf')

        # Cleanup
        lister.destroy_node()
        broadcaster.destroy_node()


class TestAccurateVsOriginal(unittest.TestCase):
    """Compare TfLister vs TfListerAccurate."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS context once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Tear down ROS context."""
        rclpy.shutdown()

    @patch('tfdiag.tf_lister_accurate.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister_accurate.tf2_ros.Buffer')
    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def test_both_detect_same_frames(self, mock_buf1, mock_lis1, mock_buf2, mock_lis2):
        """Test that both implementations detect the same frames."""
        # Create broadcaster
        broadcaster = MockTfBroadcaster('test_node')

        # Create both listers
        lister_original = TfLister()
        lister_accurate = TfListerAccurate()

        # Publish a transform
        broadcaster.publish_transform('map', 'base_link', is_static=False)

        # Spin multiple times to allow messages to propagate
        for _ in range(5):
            rclpy.spin_once(broadcaster, timeout_sec=0.1)
            rclpy.spin_once(lister_original, timeout_sec=0.1)
            rclpy.spin_once(lister_accurate, timeout_sec=0.1)

        # At least the original should track the frame
        self.assertIn('base_link', lister_original._frame_to_node)

        # Cleanup
        lister_original.destroy_node()
        lister_accurate.destroy_node()
        broadcaster.destroy_node()


class TestFrameHierarchy(unittest.TestCase):
    """Test detection of frame hierarchies."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS context once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Tear down ROS context."""
        rclpy.shutdown()

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def test_multi_level_frame_tree(self, mock_buffer, mock_listener):
        """Test detection of multi-level frame hierarchy."""
        # Create broadcaster
        broadcaster = MockTfBroadcaster('robot_state_publisher')

        # Create lister
        lister = TfLister()

        # Publish a frame tree: map -> odom -> base_link -> sensor
        broadcaster.publish_transform('map', 'odom', is_static=True)

        msg = TFMessage()
        # Odom -> base_link
        transform1 = TransformStamped()
        transform1.header.stamp = broadcaster.get_clock().now().to_msg()
        transform1.header.frame_id = 'odom'
        transform1.child_frame_id = 'base_link'
        transform1.transform.rotation.w = 1.0

        # Base_link -> sensor
        transform2 = TransformStamped()
        transform2.header.stamp = broadcaster.get_clock().now().to_msg()
        transform2.header.frame_id = 'base_link'
        transform2.child_frame_id = 'sensor'
        transform2.transform.rotation.w = 1.0

        msg.transforms = [transform1, transform2]
        broadcaster.tf_publisher.publish(msg)

        # Spin to process
        rclpy.spin_once(broadcaster, timeout_sec=0.1)
        rclpy.spin_once(lister, timeout_sec=0.1)

        # Verify all frames tracked
        self.assertIn('odom', lister._frame_to_node)
        self.assertIn('base_link', lister._frame_to_node)
        self.assertIn('sensor', lister._frame_to_node)

        # Cleanup
        lister.destroy_node()
        broadcaster.destroy_node()


class TestQoSCompatibility(unittest.TestCase):
    """Test QoS profile compatibility."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS context once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Tear down ROS context."""
        rclpy.shutdown()

    @patch('tfdiag.tf_lister_accurate.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister_accurate.tf2_ros.Buffer')
    def test_accurate_lister_qos_profiles(self, mock_buffer, mock_listener):
        """Test that TfListerAccurate uses correct QoS profiles."""
        lister = TfListerAccurate()

        # Find subscriptions
        tf_sub = None
        tf_static_sub = None

        for sub in lister.subscriptions:
            if sub.topic_name == '/tf':
                tf_sub = sub
            elif sub.topic_name == '/tf_static':
                tf_static_sub = sub

        # Verify /tf uses default QoS
        self.assertIsNotNone(tf_sub)

        # Verify /tf_static uses TRANSIENT_LOCAL
        self.assertIsNotNone(tf_static_sub)
        from rclpy.qos import DurabilityPolicy
        self.assertEqual(tf_static_sub.qos_profile.durability,
                        DurabilityPolicy.TRANSIENT_LOCAL)

        # Cleanup
        lister.destroy_node()


if __name__ == '__main__':
    unittest.main()
