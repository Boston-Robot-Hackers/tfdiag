#!/usr/bin/env python3
# test_tf_lister_accurate.py - Unit tests for TfListerAccurate
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import unittest
from unittest.mock import Mock, MagicMock, patch, call
import pytest

import rclpy
from rclpy.qos import DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from ros2diag.tf_lister_accurate import TfListerAccurate


class TestTfListerAccurateInit(unittest.TestCase):
    """Test initialization of TfListerAccurate."""

    @patch('tfdiag.tf_lister_accurate.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister_accurate.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfListerAccurate()

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    def test_node_name(self):
        """Test that node is created with correct name."""
        self.assertEqual(self.lister.get_name(), 'tf_lister')

    def test_data_structures_initialized(self):
        """Test that internal data structures are initialized."""
        self.assertIsInstance(self.lister._frame_to_gid, dict)
        self.assertIsInstance(self.lister._gid_to_node, dict)
        self.assertIsInstance(self.lister._last_message_info, dict)
        self.assertEqual(len(self.lister._frame_to_gid), 0)
        self.assertEqual(len(self.lister._gid_to_node), 0)

    def test_subscriptions_created(self):
        """Test that subscriptions are created for /tf and /tf_static."""
        # Get subscription topics
        subscriptions = self.lister.subscriptions
        topics = [sub.topic_name for sub in subscriptions]

        self.assertIn('/tf', topics)
        self.assertIn('/tf_static', topics)

    def test_tf_static_qos_has_transient_local(self):
        """Test that /tf_static subscription uses TRANSIENT_LOCAL durability."""
        # Find the /tf_static subscription
        tf_static_sub = None
        for sub in self.lister.subscriptions:
            if sub.topic_name == '/tf_static':
                tf_static_sub = sub
                break

        self.assertIsNotNone(tf_static_sub)
        # Check QoS durability
        qos_profile = tf_static_sub.qos_profile
        self.assertEqual(qos_profile.durability, DurabilityPolicy.TRANSIENT_LOCAL)


class TestGidToString(unittest.TestCase):
    """Test _gid_to_string method."""

    @patch('tfdiag.tf_lister_accurate.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister_accurate.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfListerAccurate()

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    def test_bytes_to_tuple(self):
        """Test conversion of bytes to tuple."""
        gid = b'\x01\x02\x03\x04'
        result = self.lister._gid_to_string(gid)
        self.assertIsInstance(result, tuple)
        self.assertEqual(result, (1, 2, 3, 4))

    def test_bytearray_to_tuple(self):
        """Test conversion of bytearray to tuple."""
        gid = bytearray([1, 2, 3, 4])
        result = self.lister._gid_to_string(gid)
        self.assertIsInstance(result, tuple)
        self.assertEqual(result, (1, 2, 3, 4))

    def test_list_to_tuple(self):
        """Test conversion of list to tuple (bug fix test)."""
        gid = [1, 2, 3, 4]
        result = self.lister._gid_to_string(gid)
        self.assertIsInstance(result, tuple)
        self.assertEqual(result, (1, 2, 3, 4))

    def test_list_is_hashable(self):
        """Test that converted list result is hashable."""
        gid = [1, 2, 3, 4]
        result = self.lister._gid_to_string(gid)
        # This should not raise TypeError
        test_dict = {result: 'test'}
        self.assertEqual(test_dict[result], 'test')

    def test_tuple_passthrough(self):
        """Test that tuples pass through unchanged."""
        gid = (1, 2, 3, 4)
        result = self.lister._gid_to_string(gid)
        self.assertIsInstance(result, tuple)
        self.assertEqual(result, gid)

    def test_other_types_passthrough(self):
        """Test that other types pass through unchanged."""
        gid = "some_string"
        result = self.lister._gid_to_string(gid)
        self.assertEqual(result, gid)


class TestCallbacks(unittest.TestCase):
    """Test TF callback methods."""

    @patch('tfdiag.tf_lister_accurate.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister_accurate.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfListerAccurate()

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
        """Test that _tf_callback_with_info tracks frames."""
        msg = TFMessage()
        msg.transforms = [self._create_transform_msg('test_frame')]

        self.lister._tf_callback_with_info(msg, '/tf')

        self.assertIn('test_frame', self.lister._frame_to_gid)
        self.assertEqual(self.lister._frame_to_gid['test_frame'], '/tf')

    def test_tf_static_callback_tracks_frame(self):
        """Test that _tf_static_callback_with_info tracks frames."""
        msg = TFMessage()
        msg.transforms = [self._create_transform_msg('static_frame')]

        self.lister._tf_static_callback_with_info(msg, '/tf_static')

        self.assertIn('static_frame', self.lister._frame_to_gid)
        self.assertEqual(self.lister._frame_to_gid['static_frame'], '/tf_static')

    def test_callback_multiple_frames(self):
        """Test callback with multiple frames in one message."""
        msg = TFMessage()
        msg.transforms = [
            self._create_transform_msg('frame1'),
            self._create_transform_msg('frame2'),
            self._create_transform_msg('frame3')
        ]

        self.lister._tf_callback_with_info(msg, '/tf')

        self.assertIn('frame1', self.lister._frame_to_gid)
        self.assertIn('frame2', self.lister._frame_to_gid)
        self.assertIn('frame3', self.lister._frame_to_gid)

    def test_callback_doesnt_overwrite_existing(self):
        """Test that callbacks don't overwrite existing frame mappings."""
        # First callback
        msg1 = TFMessage()
        msg1.transforms = [self._create_transform_msg('test_frame')]
        self.lister._tf_callback_with_info(msg1, '/tf')

        # Second callback with same frame but different topic
        msg2 = TFMessage()
        msg2.transforms = [self._create_transform_msg('test_frame')]
        self.lister._tf_static_callback_with_info(msg2, '/tf_static')

        # Should still be /tf (first one wins)
        self.assertEqual(self.lister._frame_to_gid['test_frame'], '/tf')


class TestBuildGidToNodeMap(unittest.TestCase):
    """Test _build_gid_to_node_map method."""

    @patch('tfdiag.tf_lister_accurate.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister_accurate.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfListerAccurate()

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    def _create_mock_publisher_info(self, node_name, node_namespace, endpoint_gid):
        """Helper to create mock publisher info."""
        info = Mock()
        info.node_name = node_name
        info.node_namespace = node_namespace
        info.endpoint_gid = endpoint_gid
        return info

    def test_build_gid_map_with_bytes(self):
        """Test building GID map with bytes GIDs."""
        mock_pub = self._create_mock_publisher_info(
            'test_node', '/namespace/', b'\x01\x02\x03\x04'
        )

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=[mock_pub]):
            self.lister._build_gid_to_node_map()

        gid_key = (1, 2, 3, 4)
        self.assertIn(gid_key, self.lister._gid_to_node)
        self.assertEqual(self.lister._gid_to_node[gid_key], '/namespace/test_node')

    def test_build_gid_map_with_list(self):
        """Test building GID map with list GIDs (bug fix test)."""
        mock_pub = self._create_mock_publisher_info(
            'test_node', '/', [1, 2, 3, 4]
        )

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=[mock_pub]):
            # This should not raise TypeError
            self.lister._build_gid_to_node_map()

        gid_key = (1, 2, 3, 4)
        self.assertIn(gid_key, self.lister._gid_to_node)
        self.assertEqual(self.lister._gid_to_node[gid_key], '/test_node')

    def test_build_gid_map_multiple_publishers(self):
        """Test building GID map with multiple publishers."""
        mock_pubs = [
            self._create_mock_publisher_info('node1', '/', [1, 2, 3, 4]),
            self._create_mock_publisher_info('node2', '/ns/', [5, 6, 7, 8])
        ]

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=mock_pubs):
            self.lister._build_gid_to_node_map()

        # Should have 2 unique GIDs (one per publisher), method is called for both /tf and /tf_static
        # but same publishers returned, so still only 2 unique GIDs
        self.assertEqual(len(self.lister._gid_to_node), 2)
        self.assertIn((1, 2, 3, 4), self.lister._gid_to_node)
        self.assertIn((5, 6, 7, 8), self.lister._gid_to_node)


class TestGetFramePublishersByTopic(unittest.TestCase):
    """Test get_frame_publishers_by_topic method."""

    @patch('tfdiag.tf_lister_accurate.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister_accurate.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfListerAccurate()

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    def _create_mock_publisher_info(self, node_name, node_namespace, endpoint_gid):
        """Helper to create mock publisher info."""
        info = Mock()
        info.node_name = node_name
        info.node_namespace = node_namespace
        info.endpoint_gid = endpoint_gid
        return info

    def test_map_frames_to_tf_nodes(self):
        """Test mapping frames to /tf publisher nodes."""
        # Set up frame tracking
        self.lister._frame_to_gid['test_frame'] = '/tf'

        # Mock publishers
        mock_pub = self._create_mock_publisher_info('robot_state_publisher', '/', [1, 2, 3, 4])

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=[mock_pub]):
            result = self.lister.get_frame_publishers_by_topic()

        self.assertIn('test_frame', result)
        self.assertIn('/robot_state_publisher', result['test_frame'])

    def test_map_frames_to_tf_static_nodes(self):
        """Test mapping frames to /tf_static publisher nodes."""
        # Set up frame tracking
        self.lister._frame_to_gid['static_frame'] = '/tf_static'

        # Mock publishers
        mock_pub = self._create_mock_publisher_info('static_broadcaster', '/', [1, 2, 3, 4])

        def mock_get_pubs(topic):
            if topic == '/tf_static':
                return [mock_pub]
            return []

        with patch.object(self.lister, 'get_publishers_info_by_topic', side_effect=mock_get_pubs):
            result = self.lister.get_frame_publishers_by_topic()

        self.assertIn('static_frame', result)
        self.assertIn('/static_broadcaster', result['static_frame'])

    def test_mixed_tf_and_static_frames(self):
        """Test with both /tf and /tf_static frames."""
        # Set up frame tracking
        self.lister._frame_to_gid['dynamic_frame'] = '/tf'
        self.lister._frame_to_gid['static_frame'] = '/tf_static'

        # Mock publishers
        dynamic_pub = self._create_mock_publisher_info('dynamic_node', '/', [1, 2, 3, 4])
        static_pub = self._create_mock_publisher_info('static_node', '/', [5, 6, 7, 8])

        def mock_get_pubs(topic):
            if topic == '/tf':
                return [dynamic_pub]
            elif topic == '/tf_static':
                return [static_pub]
            return []

        with patch.object(self.lister, 'get_publishers_info_by_topic', side_effect=mock_get_pubs):
            result = self.lister.get_frame_publishers_by_topic()

        self.assertIn('dynamic_frame', result)
        self.assertIn('static_frame', result)
        self.assertIn('/dynamic_node', result['dynamic_frame'])
        self.assertIn('/static_node', result['static_frame'])


class TestGetAllBroadcasterNodes(unittest.TestCase):
    """Test get_all_broadcaster_nodes method."""

    @patch('tfdiag.tf_lister_accurate.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister_accurate.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfListerAccurate()

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    def _create_mock_publisher_info(self, node_name, node_namespace, endpoint_gid):
        """Helper to create mock publisher info."""
        info = Mock()
        info.node_name = node_name
        info.node_namespace = node_namespace
        info.endpoint_gid = endpoint_gid
        return info

    def test_get_unique_broadcaster_nodes(self):
        """Test extraction of unique broadcaster nodes."""
        mock_pubs = [
            self._create_mock_publisher_info('node1', '/', [1, 2, 3, 4]),
            self._create_mock_publisher_info('node2', '/ns/', [5, 6, 7, 8])
        ]

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=mock_pubs):
            result = self.lister.get_all_broadcaster_nodes()

        self.assertIsInstance(result, set)
        self.assertIn('/node1', result)
        self.assertIn('/ns/node2', result)

    def test_node_on_both_topics_counted_once(self):
        """Test that a node publishing to both topics appears only once."""
        mock_pub = self._create_mock_publisher_info('multi_node', '/', [1, 2, 3, 4])

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=[mock_pub]):
            result = self.lister.get_all_broadcaster_nodes()

        # Count occurrences
        node_list = list(result)
        self.assertEqual(node_list.count('/multi_node'), 1)


class TestGetAllFramesInfo(unittest.TestCase):
    """Test get_all_frames_info method."""

    @patch('tfdiag.tf_lister_accurate.tf2_ros.TransformListener')
    @patch('tfdiag.tf_lister_accurate.tf2_ros.Buffer')
    def setUp(self, mock_buffer, mock_listener):
        """Set up test fixtures."""
        rclpy.init()
        self.lister = TfListerAccurate()
        self.mock_buffer = mock_buffer

    def tearDown(self):
        """Clean up after tests."""
        self.lister.destroy_node()
        rclpy.shutdown()

    @patch('rclpy.spin_once')
    def test_empty_frames(self, mock_spin):
        """Test with no TF frames."""
        self.lister._tf_buffer.all_frames_as_yaml.return_value = ''

        frames_dict, frame_map, broadcasters = self.lister.get_all_frames_info()

        self.assertEqual(frames_dict, {})
        self.assertEqual(frame_map, {})
        self.assertEqual(broadcasters, set())

    @patch('rclpy.spin_once')
    def test_static_frame_detection(self, mock_spin):
        """Test that static frames are detected by high rate."""
        yaml_data = """
map:
  parent: ''
  rate: 10000.0
"""
        self.lister._tf_buffer.all_frames_as_yaml.return_value = yaml_data

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=[]):
            frames_dict, frame_map, broadcasters = self.lister.get_all_frames_info()

        # Verify the frame was added to tracking as static
        self.assertIn('map', self.lister._frame_to_gid)
        self.assertEqual(self.lister._frame_to_gid['map'], '/tf_static')

    @patch('rclpy.spin_once')
    def test_dynamic_frame_detection(self, mock_spin):
        """Test that dynamic frames are detected by low rate."""
        yaml_data = """
base_link:
  parent: 'odom'
  rate: 10.0
"""
        self.lister._tf_buffer.all_frames_as_yaml.return_value = yaml_data

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=[]):
            frames_dict, frame_map, broadcasters = self.lister.get_all_frames_info()

        # Verify the frame was added to tracking as dynamic
        self.assertIn('base_link', self.lister._frame_to_gid)
        self.assertEqual(self.lister._frame_to_gid['base_link'], '/tf')

    @patch('rclpy.spin_once')
    def test_mixed_static_and_dynamic(self, mock_spin):
        """Test with both static and dynamic frames."""
        yaml_data = """
map:
  parent: ''
  rate: 10000.0
odom:
  parent: 'map'
  rate: 50.0
base_link:
  parent: 'odom'
  rate: 10.0
"""
        self.lister._tf_buffer.all_frames_as_yaml.return_value = yaml_data

        with patch.object(self.lister, 'get_publishers_info_by_topic', return_value=[]):
            frames_dict, frame_map, broadcasters = self.lister.get_all_frames_info()

        # Verify correct categorization
        self.assertEqual(self.lister._frame_to_gid['map'], '/tf_static')
        self.assertEqual(self.lister._frame_to_gid['odom'], '/tf')
        self.assertEqual(self.lister._frame_to_gid['base_link'], '/tf')

        # Verify return values
        self.assertEqual(len(frames_dict), 3)
        self.assertIn('map', frames_dict)
        self.assertIn('odom', frames_dict)
        self.assertIn('base_link', frames_dict)


if __name__ == '__main__':
    unittest.main()
