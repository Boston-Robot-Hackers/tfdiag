#!/usr/bin/env python3
# test_tf_lister.py - Unit tests for TfLister
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import unittest
from unittest.mock import Mock, MagicMock, patch
import pytest

import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from tfdiag.tf_lister import TfLister


class TestTfListerInit(unittest.TestCase):
    """Test initialization of TfLister."""

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfLister()

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    def test_node_name(self):
        """Test that node is created with correct name."""
        self.assertEqual(self.lister.get_name(), 'tf_lister')

    def test_data_structures_initialized(self):
        """Test that internal data structures are initialized."""
        self.assertIsInstance(self.lister._broadcaster_nodes, set)
        self.assertIsInstance(self.lister._frame_to_node, dict)
        self.assertIsInstance(self.lister._frame_to_node_map, dict)
        self.assertEqual(len(self.lister._broadcaster_nodes), 0)
        self.assertEqual(len(self.lister._frame_to_node), 0)

    def test_subscriptions_created(self):
        """Test that subscriptions are created for /tf and /tf_static."""
        subscriptions = self.lister.subscriptions
        topics = [sub.topic_name for sub in subscriptions]

        self.assertIn('/tf', topics)
        self.assertIn('/tf_static', topics)


class TestCallbacks(unittest.TestCase):
    """Test TF callback methods."""

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfLister()

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    def _create_transform_msg(self, child_frame_id):
        """Helper to create a TransformStamped message."""
        transform = TransformStamped()
        transform.child_frame_id = child_frame_id
        transform.header.frame_id = 'parent_frame'
        return transform

    def test_tf_callback_tracks_frame(self):
        """Test that _tf_callback tracks frames on /tf."""
        msg = TFMessage()
        msg.transforms = [self._create_transform_msg('test_frame')]

        self.lister._tf_callback(msg)

        self.assertIn('test_frame', self.lister._frame_to_node)
        self.assertEqual(self.lister._frame_to_node['test_frame'], '/tf')

    def test_tf_static_callback_tracks_frame(self):
        """Test that _tf_static_callback tracks frames on /tf_static."""
        msg = TFMessage()
        msg.transforms = [self._create_transform_msg('static_frame')]

        self.lister._tf_static_callback(msg)

        self.assertIn('static_frame', self.lister._frame_to_node)
        self.assertEqual(self.lister._frame_to_node['static_frame'], '/tf_static')

    def test_callback_multiple_frames(self):
        """Test callback with multiple frames in one message."""
        msg = TFMessage()
        msg.transforms = [
            self._create_transform_msg('frame1'),
            self._create_transform_msg('frame2'),
            self._create_transform_msg('frame3')
        ]

        self.lister._tf_callback(msg)

        self.assertIn('frame1', self.lister._frame_to_node)
        self.assertIn('frame2', self.lister._frame_to_node)
        self.assertIn('frame3', self.lister._frame_to_node)
        # All should be mapped to /tf
        self.assertEqual(self.lister._frame_to_node['frame1'], '/tf')
        self.assertEqual(self.lister._frame_to_node['frame2'], '/tf')
        self.assertEqual(self.lister._frame_to_node['frame3'], '/tf')

    def test_callback_doesnt_overwrite_existing(self):
        """Test that callbacks don't overwrite existing frame mappings."""
        # First message on /tf
        msg1 = TFMessage()
        msg1.transforms = [self._create_transform_msg('test_frame')]
        self.lister._tf_callback(msg1)

        # Second message on /tf_static with same frame
        msg2 = TFMessage()
        msg2.transforms = [self._create_transform_msg('test_frame')]
        self.lister._tf_static_callback(msg2)

        # Should still be /tf (first one wins)
        self.assertEqual(self.lister._frame_to_node['test_frame'], '/tf')


class TestGetBroadcasterNodes(unittest.TestCase):
    """Test get_broadcaster_nodes method."""

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfLister()

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    def _create_mock_publisher_info(self, node_name, node_namespace):
        """Helper to create mock publisher info."""
        info = Mock()
        info.node_name = node_name
        info.node_namespace = node_namespace
        return info

    def test_get_tf_publishers(self):
        """Test extraction of /tf publisher nodes."""
        mock_pubs = [
            self._create_mock_publisher_info('node1', '/'),
            self._create_mock_publisher_info('node2', '/namespace/')
        ]

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=mock_pubs):
            broadcasters, tf_nodes, tf_static_nodes = self.lister.get_broadcaster_nodes()

        self.assertIn('/node1', broadcasters)
        self.assertIn('/namespace/node2', broadcasters)
        self.assertEqual(len(tf_nodes), 2)

    def test_get_tf_static_publishers(self):
        """Test extraction of /tf_static publisher nodes."""
        mock_tf_pubs = []
        mock_tf_static_pubs = [
            self._create_mock_publisher_info('static_node', '/')
        ]

        def mock_get_pubs(topic):
            if topic == '/tf':
                return mock_tf_pubs
            elif topic == '/tf_static':
                return mock_tf_static_pubs
            return []

        with patch.object(self.lister, 'get_publishers_info_by_topic', side_effect=mock_get_pubs):
            broadcasters, tf_nodes, tf_static_nodes = self.lister.get_broadcaster_nodes()

        self.assertIn('/static_node', broadcasters)
        self.assertEqual(len(tf_static_nodes), 1)
        self.assertEqual(len(tf_nodes), 0)

    def test_node_on_both_topics(self):
        """Test node publishing to both /tf and /tf_static."""
        mock_pub = self._create_mock_publisher_info('multi_node', '/')

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=[mock_pub]):
            broadcasters, tf_nodes, tf_static_nodes = self.lister.get_broadcaster_nodes()

        # Should appear in broadcaster set only once
        self.assertIn('/multi_node', broadcasters)
        # Should appear in both topic maps
        self.assertIn('/multi_node', tf_nodes)
        self.assertIn('/multi_node', tf_static_nodes)


class TestMapFramesToNodes(unittest.TestCase):
    """Test map_frames_to_nodes method."""

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfLister()

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    def test_map_tf_frame_to_nodes(self):
        """Test mapping a /tf frame to publisher nodes."""
        self.lister._frame_to_node['test_frame'] = '/tf'
        tf_nodes = {'/node1': Mock(), '/node2': Mock()}
        tf_static_nodes = {}

        result = self.lister.map_frames_to_nodes(tf_nodes, tf_static_nodes, ['test_frame'])

        self.assertIn('test_frame', result)
        self.assertIn('/node1', result['test_frame'])
        self.assertIn('/node2', result['test_frame'])

    def test_map_tf_static_frame_to_nodes(self):
        """Test mapping a /tf_static frame to publisher nodes."""
        self.lister._frame_to_node['static_frame'] = '/tf_static'
        tf_nodes = {}
        tf_static_nodes = {'/static_node': Mock()}

        result = self.lister.map_frames_to_nodes(tf_nodes, tf_static_nodes, ['static_frame'])

        self.assertIn('static_frame', result)
        self.assertIn('/static_node', result['static_frame'])

    def test_map_unknown_frame(self):
        """Test mapping a frame not seen in callbacks."""
        tf_nodes = {}
        tf_static_nodes = {'/static_node': Mock()}

        result = self.lister.map_frames_to_nodes(tf_nodes, tf_static_nodes, ['unknown_frame'])

        # Unknown frames default to tf_static nodes
        self.assertIn('unknown_frame', result)
        self.assertIn('/static_node', result['unknown_frame'])

    def test_map_multiple_frames(self):
        """Test mapping multiple frames."""
        self.lister._frame_to_node['frame1'] = '/tf'
        self.lister._frame_to_node['frame2'] = '/tf_static'
        tf_nodes = {'/dynamic_node': Mock()}
        tf_static_nodes = {'/static_node': Mock()}

        all_frames = ['frame1', 'frame2', 'unknown_frame']
        result = self.lister.map_frames_to_nodes(tf_nodes, tf_static_nodes, all_frames)

        self.assertEqual(len(result), 3)
        self.assertIn('/dynamic_node', result['frame1'])
        self.assertIn('/static_node', result['frame2'])
        self.assertIn('/static_node', result['unknown_frame'])


class TestGetAllFramesInfo(unittest.TestCase):
    """Test get_all_frames_info method."""

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfLister()
        self.mock_buffer = mock_buffer

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    @patch('rclpy.spin_once')
    def test_no_frames(self, mock_spin):
        """Test with no TF frames."""
        self.lister._tf_buffer.all_frames_as_yaml.return_value = ''

        result = self.lister.get_all_frames_info()

        self.assertEqual(result, {})

    @patch('rclpy.spin_once')
    def test_with_frames(self, mock_spin):
        """Test with TF frames in buffer."""
        yaml_data = """
map:
  parent: ''
  rate: 10000.0
odom:
  parent: 'map'
  rate: 50.0
"""
        self.lister._tf_buffer.all_frames_as_yaml.return_value = yaml_data

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=[]):
            result = self.lister.get_all_frames_info()

        self.assertIsInstance(result, dict)
        self.assertIn('map', result)
        self.assertIn('odom', result)
        # Verify frame_to_node_map was populated
        self.assertIsInstance(self.lister._frame_to_node_map, dict)


class TestPrintFramesInfo(unittest.TestCase):
    """Test print_frames_info method."""

    @patch('tfdiag.tf_lister.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfLister()

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    @patch('builtins.print')
    @patch('rclpy.spin_once')
    def test_print_no_frames(self, mock_spin, mock_print):
        """Test printing when no frames exist."""
        self.lister._tf_buffer.all_frames_as_yaml.return_value = ''

        self.lister.print_frames_info()

        # Should print "No TF frames found"
        print_calls = [str(call) for call in mock_print.call_args_list]
        self.assertTrue(any('No TF frames found' in call for call in print_calls))

    @patch('builtins.print')
    @patch('rclpy.spin_once')
    def test_print_with_frames(self, mock_spin, mock_print):
        """Test printing with frames."""
        yaml_data = """
map:
  parent: ''
  rate: 10000.0
  buffer_length: 10.0
  most_recent_transform: 1.0
"""
        self.lister._tf_buffer.all_frames_as_yaml.return_value = yaml_data
        self.lister._frame_to_node['map'] = '/tf_static'
        self.lister._frame_to_node_map = {'map': ['/static_broadcaster']}
        self.lister._broadcaster_nodes = {'/static_broadcaster'}

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=[]):
            self.lister.print_frames_info()

        # Verify various parts were printed
        print_calls = [str(call) for call in mock_print.call_args_list]
        frame_printed = any('Frame: map' in call for call in print_calls)
        static_label = any('static frame' in call for call in print_calls)
        broadcaster_printed = any('Broadcaster Nodes' in call for call in print_calls)

        self.assertTrue(frame_printed)
        self.assertTrue(static_label)
        self.assertTrue(broadcaster_printed)


if __name__ == '__main__':
    unittest.main()
