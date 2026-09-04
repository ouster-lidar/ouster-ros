# SPDX-License-Identifier: BSD-3-Clause

"""End-to-end packet tests for the ROS camera processing nodes."""

import math
import struct
import subprocess
import time
import unittest
from pathlib import Path

import launch

import launch_ros.actions

import launch_testing
import launch_testing.actions

try:
    from launch_testing_ros.actions import EnableRmwIsolation
except ImportError:
    # Humble predates this action. The test still uses dedicated names and
    # topics there, while newer distributions also isolate the RMW graph.
    EnableRmwIsolation = None

from ouster_sensor_msgs.msg import PacketMsg

import pytest

import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from std_msgs.msg import String

from tf2_msgs.msg import TFMessage


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PCAP_ROOT = PACKAGE_ROOT / 'ouster-sdk' / 'tests' / 'pcaps'
METADATA_PATH = PCAP_ROOT / 'OS-0-32-U1_v2.2.0_1024x10.json'
PCAP_PATH = PCAP_ROOT / 'OS-0-32-U1_v2.2.0_1024x10.pcap'
LIDAR_PORT = 7502


def _camera_node(executable, namespace, name, parameters, remappings=None):
    return launch_ros.actions.Node(
        package='ouster_ros',
        executable=executable,
        namespace=namespace,
        name=name,
        parameters=[parameters],
        remappings=remappings or [],
        output='screen',
    )


@pytest.mark.launch_test
def generate_test_description():
    common = {
        'timestamp_mode': 'TIME_FROM_INTERNAL_OSC',
        'use_system_default_qos': True,
    }
    pinhole = _camera_node(
        'os_pinhole',
        'camera_test',
        'os_pinhole',
        {
            **common,
            'panel_names': ['front'],
            'panel_yaws_deg': [0.0],
            'panel_pitches_deg': [10.0],
            'panel_hfovs_deg': [90.0],
            'panel_widths': [64],
            'panel_heights': [32],
            'panel_vfovs_deg': [60.0],
            'panel_roi_x_offsets': [8],
            'panel_roi_y_offsets': [12],
            'panel_roi_widths': [48],
            'panel_roi_heights': [12],
            'publish_signal_image': False,
            'publish_reflec_image2': False,
            'parent_frame': 'test_lidar',
        },
    )
    image_enabled = _camera_node(
        'os_image',
        'panorama_enabled',
        'os_image',
        {
            **common,
            'publish_camera_info': True,
            'frame_id': 'test_lidar',
        },
        [
            ('lidar_packets', '/camera_test/lidar_packets'),
            ('metadata', '/camera_test/metadata'),
        ],
    )
    image_default = _camera_node(
        'os_image',
        'panorama_default',
        'os_image',
        {
            **common,
            'frame_id': 'test_lidar_default',
        },
        [
            ('lidar_packets', '/camera_test/lidar_packets'),
            ('metadata', '/camera_test/metadata'),
        ],
    )

    actions = [pinhole, image_enabled, image_default]
    if EnableRmwIsolation is not None:
        actions.insert(0, EnableRmwIsolation())
    actions.append(launch_testing.actions.ReadyToTest())

    return (
        launch.LaunchDescription(actions),
        {
            'pinhole': pinhole,
            'image_enabled': image_enabled,
            'image_default': image_default,
        },
    )


def _udp_payloads(pcap_path, destination_port):
    """Read UDP payloads from the Ethernet/IPv4 test capture."""
    capture = Path(pcap_path).read_bytes()
    endian_by_magic = {
        b'\xd4\xc3\xb2\xa1': '<',
        b'\xa1\xb2\xc3\xd4': '>',
        b'M<\xb2\xa1': '<',
        b'\xa1\xb2<M': '>',
    }
    if len(capture) < 24 or capture[:4] not in endian_by_magic:
        raise ValueError('unsupported or truncated pcap header')
    endian = endian_by_magic[capture[:4]]
    link_type = struct.unpack_from(endian + 'I', capture, 20)[0]
    if link_type != 1:
        raise ValueError('test fixture must use Ethernet link-layer packets')

    payloads = []
    offset = 24
    while offset + 16 <= len(capture):
        _, _, included_length, _ = struct.unpack_from(
            endian + 'IIII', capture, offset)
        offset += 16
        frame_end = offset + included_length
        if frame_end > len(capture):
            raise ValueError('truncated pcap packet record')
        frame = capture[offset:frame_end]
        offset = frame_end

        if len(frame) < 14:
            continue
        ether_type = struct.unpack_from('!H', frame, 12)[0]
        ip_offset = 14
        if ether_type == 0x8100 and len(frame) >= 18:
            ether_type = struct.unpack_from('!H', frame, 16)[0]
            ip_offset = 18
        if ether_type != 0x0800 or len(frame) < ip_offset + 20:
            continue

        ip_header_length = (frame[ip_offset] & 0x0f) * 4
        if ip_header_length < 20 or frame[ip_offset + 9] != 17:
            continue
        udp_offset = ip_offset + ip_header_length
        if len(frame) < udp_offset + 8:
            continue
        _, dst_port, udp_length, _ = struct.unpack_from(
            '!HHHH', frame, udp_offset)
        if dst_port != destination_port or udp_length < 8:
            continue
        payload_end = udp_offset + udp_length
        if payload_end > len(frame):
            raise ValueError('truncated UDP payload in pcap fixture')
        payloads.append(frame[udp_offset + 8:payload_end])

    if offset != len(capture):
        raise ValueError('trailing bytes in pcap fixture')
    return payloads


def _stamp(message):
    return (message.header.stamp.sec, message.header.stamp.nanosec)


def _rotated_optical_z(transform):
    """Return where the child optical +Z axis points in its parent frame."""
    q = transform.rotation
    return (
        2.0 * (q.x * q.z + q.w * q.y),
        2.0 * (q.y * q.z - q.w * q.x),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )


class TestCameraNodes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('camera_nodes_launch_tester')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout_sec, failure_message):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail(failure_message)

    def test_modified_launch_files_parse(self):
        launch_arguments = {
            'pinhole.launch.py': ('ouster_ns', 'params_file'),
            'record.composite.launch.xml': (
                'publish_camera_info', 'camera_optical_frame'),
            'replay.composite.launch.xml': (
                'publish_camera_info', 'camera_optical_frame'),
            'replay_pcap.launch.xml': (
                'publish_camera_info', 'camera_optical_frame'),
        }
        for launch_file, expected_arguments in launch_arguments.items():
            with self.subTest(launch_file=launch_file):
                result = subprocess.run(
                    [
                        'ros2', 'launch', 'ouster_ros', launch_file,
                        '--show-args',
                    ],
                    capture_output=True,
                    check=False,
                    encoding='utf-8',
                    timeout=20.0,
                )
                self.assertEqual(
                    0,
                    result.returncode,
                    '{} failed to parse:\n{}\n{}'.format(
                        launch_file, result.stdout, result.stderr),
                )
                for argument in expected_arguments:
                    self.assertIn("'{}'".format(argument), result.stdout)

    def test_packet_to_camera_outputs_and_static_transforms(self):
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=128,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        received = {}
        received_counts = {}
        transforms = {}
        transform_counts = {}
        subscriptions = []

        def save(key):
            def callback(message):
                received.setdefault(key, message)
                received_counts[key] = received_counts.get(key, 0) + 1
            return callback

        image_topics = {
            'pinhole_range': '/camera_test/panels/front/range_image',
            'pinhole_depth': '/camera_test/panels/front/depth_image',
            'pinhole_depth2': '/camera_test/panels/front/depth_image2',
            'panorama_enabled': '/panorama_enabled/range_image',
            'panorama_default': '/panorama_default/range_image',
        }
        for key, topic in image_topics.items():
            subscriptions.append(
                self.node.create_subscription(
                    Image, topic, save(key), reliable_qos)
            )
        subscriptions.extend([
            self.node.create_subscription(
                CameraInfo,
                '/camera_test/panels/front/camera_info',
                save('pinhole_info'),
                reliable_qos,
            ),
            self.node.create_subscription(
                CameraInfo,
                '/panorama_enabled/camera_info',
                save('panorama_info'),
                reliable_qos,
            ),
        ])

        def save_transforms(message):
            for transform in message.transforms:
                key = (transform.header.frame_id,
                       transform.child_frame_id)
                transforms[key] = transform.transform
                transform_counts[key] = transform_counts.get(key, 0) + 1

        subscriptions.append(
            self.node.create_subscription(
                TFMessage, '/tf_static', save_transforms, static_tf_qos)
        )

        packet_pub = self.node.create_publisher(
            PacketMsg, '/camera_test/lidar_packets', reliable_qos)
        metadata_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        metadata_pub = self.node.create_publisher(
            String, '/camera_test/metadata', metadata_qos)

        self._spin_until(
            lambda: metadata_pub.get_subscription_count() == 3,
            15.0,
            'camera nodes did not discover the metadata publisher',
        )

        active_metadata = String()
        active_metadata.data = METADATA_PATH.read_text(encoding='utf-8')
        metadata_pub.publish(active_metadata)

        expected_topics = list(image_topics.values()) + [
            '/camera_test/panels/front/camera_info',
            '/panorama_enabled/camera_info',
        ]
        self._spin_until(
            lambda: (
                packet_pub.get_subscription_count() == 3 and
                all(
                    self.node.get_publishers_info_by_topic(topic)
                    for topic in expected_topics
                )
            ),
            15.0,
            'camera nodes did not finish metadata activation and discovery',
        )
        self.assertFalse(
            self.node.get_publishers_info_by_topic(
                '/panorama_default/camera_info'),
            'os_image must not publish CameraInfo unless explicitly enabled',
        )
        self.assertFalse(
            self.node.get_publishers_info_by_topic(
                '/camera_test/panels/front/rgb_image'),
            'os_pinhole must not publish RGB unless explicitly enabled on an '
            'RGB profile',
        )
        self.assertFalse(
            self.node.get_publishers_info_by_topic(
                '/camera_test/panels/front/signal_image'),
            'a disabled first-return image must not have a publisher',
        )
        self.assertFalse(
            self.node.get_publishers_info_by_topic(
                '/camera_test/panels/front/reflec_image2'),
            'a disabled second-return image must not have a publisher',
        )

        # Receiving the same metadata again must be an idempotent no-op, not a
        # pipeline replacement.
        metadata_pub.publish(active_metadata)

        # All three nodes must reject a short packet without producing a scan.
        short_packet = PacketMsg()
        short_packet.buf = bytes(16)
        packet_pub.publish(short_packet)
        quiet_deadline = time.monotonic() + 0.4
        while time.monotonic() < quiet_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertFalse(
            received, 'an undersized packet produced camera output')

        payloads = _udp_payloads(PCAP_PATH, LIDAR_PORT)
        self.assertEqual(64, len(payloads))
        self.assertEqual({8448}, {len(payload) for payload in payloads})

        def publish_frame():
            for payload in payloads:
                packet = PacketMsg()
                packet.buf = payload
                packet_pub.publish(packet)
                time.sleep(0.002)

        publish_frame()

        expected_messages = set(image_topics) | {
            'pinhole_info', 'panorama_info'}
        self._spin_until(
            lambda: expected_messages <= received.keys(),
            15.0,
            'a complete lidar scan did not produce all camera outputs',
        )

        pinhole_info = received['pinhole_info']
        pinhole_range = received['pinhole_range']
        pinhole_depth = received['pinhole_depth']
        pinhole_depth2 = received['pinhole_depth2']
        pinhole_frame = 'camera_test/panels/front_optical_frame'
        self.assertEqual((64, 32), (pinhole_info.width, pinhole_info.height))
        self.assertEqual('plumb_bob', pinhole_info.distortion_model)
        self.assertEqual(pinhole_frame, pinhole_info.header.frame_id)
        self.assertAlmostEqual(32.0, pinhole_info.k[0], places=12)
        self.assertAlmostEqual(16.0 / math.tan(math.radians(30.0)),
                               pinhole_info.k[4], places=12)
        self.assertAlmostEqual(31.5, pinhole_info.k[2], places=12)
        self.assertAlmostEqual(15.5, pinhole_info.k[5], places=12)
        self.assertEqual((8, 12, 48, 12), (
            pinhole_info.roi.x_offset,
            pinhole_info.roi.y_offset,
            pinhole_info.roi.width,
            pinhole_info.roi.height,
        ))
        self.assertEqual('mono16', pinhole_range.encoding)
        self.assertEqual('32FC1', pinhole_depth.encoding)
        self.assertEqual('32FC1', pinhole_depth2.encoding)
        self.assertEqual((48, 12), (pinhole_depth.width,
                                    pinhole_depth.height))
        self.assertEqual(48 * 4, pinhole_depth.step)
        self.assertEqual(48 * 12 * 4, len(pinhole_depth.data))
        self.assertEqual(pinhole_frame, pinhole_range.header.frame_id)
        self.assertEqual(pinhole_frame, pinhole_depth.header.frame_id)
        self.assertEqual(pinhole_frame, pinhole_depth2.header.frame_id)
        self.assertEqual(_stamp(pinhole_info), _stamp(pinhole_range))
        self.assertEqual(_stamp(pinhole_info), _stamp(pinhole_depth))
        self.assertEqual(_stamp(pinhole_info), _stamp(pinhole_depth2))
        self.assertNotEqual((0, 0), _stamp(pinhole_info))

        byte_order = '>' if pinhole_depth.is_bigendian else '<'
        depth_values = [value[0] for value in struct.iter_unpack(
            byte_order + 'f', pinhole_depth.data)]
        depth2_values = [value[0] for value in struct.iter_unpack(
            byte_order + 'f', pinhole_depth2.data)]
        for values in (depth_values, depth2_values):
            self.assertTrue(any(math.isfinite(value) and value > 0.0
                                for value in values))
            self.assertTrue(all(
                math.isnan(value) or (math.isfinite(value) and value > 0.0)
                for value in values
            ))

        panorama_info = received['panorama_info']
        panorama_enabled = received['panorama_enabled']
        panorama_default = received['panorama_default']
        panorama_frame = 'test_lidar_panorama_optical_frame'
        self.assertEqual((1024, 32),
                         (panorama_info.width, panorama_info.height))
        self.assertEqual(panorama_frame, panorama_info.header.frame_id)
        self.assertEqual(panorama_frame, panorama_enabled.header.frame_id)
        self.assertEqual(_stamp(panorama_info), _stamp(panorama_enabled))
        self.assertEqual('test_lidar_default',
                         panorama_default.header.frame_id)

        pinhole_tf_key = ('test_lidar', pinhole_frame)
        panorama_tf_key = ('test_lidar', panorama_frame)
        self._spin_until(
            lambda: (
                pinhole_tf_key in transforms and
                panorama_tf_key in transforms
            ),
            10.0,
            'camera optical transforms were not published on /tf_static',
        )
        pinhole_forward = _rotated_optical_z(transforms[pinhole_tf_key])
        panorama_forward = _rotated_optical_z(transforms[panorama_tf_key])
        pitch = math.radians(10.0)
        for actual, expected in zip(
                pinhole_forward, (math.cos(pitch), 0.0, math.sin(pitch))):
            self.assertAlmostEqual(expected, actual, places=6)
        for actual, expected in zip(panorama_forward, (-1.0, 0.0, 0.0)):
            self.assertAlmostEqual(expected, actual, places=6)
        self.assertNotIn(
            ('test_lidar_default',
             'test_lidar_default_panorama_optical_frame'),
            transforms,
        )

        # A bad metadata update must be contained. A subsequent valid update
        # must rebuild the handler; replaying the same frame then produces a
        # second set of pinhole messages instead of being rejected as a
        # duplicate by the old ScanBatcher.
        first_counts = {
            key: received_counts[key]
            for key in ('pinhole_range', 'pinhole_depth', 'pinhole_info')
        }
        first_pinhole_tf_count = transform_counts[pinhole_tf_key]
        invalid_metadata = String()
        invalid_metadata.data = '{not valid sensor metadata'
        metadata_pub.publish(invalid_metadata)
        self._spin_until(
            lambda: packet_pub.get_subscription_count() == 0,
            10.0,
            'camera nodes kept a stale packet pipeline after bad metadata',
        )
        inactive_counts = dict(received_counts)
        publish_frame()
        quiet_deadline = time.monotonic() + 0.4
        while time.monotonic() < quiet_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(
            inactive_counts,
            dict(received_counts),
            'camera output continued while metadata was invalid',
        )
        metadata_pub.publish(active_metadata)
        self._spin_until(
            lambda: transform_counts.get(pinhole_tf_key, 0) >
            first_pinhole_tf_count,
            15.0,
            'os_pinhole did not rebuild after valid metadata',
        )
        publish_frame()
        self._spin_until(
            lambda: all(
                received_counts.get(key, 0) > count
                for key, count in first_counts.items()
            ),
            15.0,
            'os_pinhole did not recover after a malformed metadata update',
        )

        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)
        self.node.destroy_publisher(packet_pub)
        self.node.destroy_publisher(metadata_pub)


@launch_testing.post_shutdown_test()
class TestCameraNodeShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(
            self, proc_info, pinhole, image_enabled, image_default):
        for process in (pinhole, image_enabled, image_default):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=[0], process=process)
