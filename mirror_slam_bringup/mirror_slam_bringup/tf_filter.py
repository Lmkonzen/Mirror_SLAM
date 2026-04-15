#!/usr/bin/env python3
"""Relay /tf → /tf_prefixed, adding ``arm_`` prefix to all UR frames.

The UR driver and TurtleBot3 both publish ``base_link`` to /tf.  This
node subscribes to the raw /tf, identifies UR-specific frames, adds
the ``arm_`` prefix, and republishes everything to /tf_prefixed.
Non-UR frames pass through unchanged.

Result on /tf_prefixed:
    TB3 chain:  map → odom → base_footprint → base_link → base_scan …
    UR chain:   arm_world → arm_base_link → arm_base_link_inertia →
                arm_shoulder_link → … → arm_tool0

MoveIt keeps working on the raw /tf with original names.
gap_explorer remaps to /tf_prefixed so it sees a single clean tree.

Usage:
    ros2 run mirror_slam_bringup tf_filter --ros-args -p use_sim_time:=true

    ros2 run gap_explorer gap_explorer --ros-args -p use_sim_time:=true \\
        -r /tf:=/tf_prefixed -r /tf_static:=/tf_static_prefixed
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

# Every frame published by the UR driver + robot_state_publisher.
# Sourced from your view_frames output.
UR_FRAMES = frozenset([
    'world',
    'base_link',
    'base_link_inertia',
    'base',
    'shoulder_link',
    'upper_arm_link',
    'forearm_link',
    'wrist_1_link',
    'wrist_2_link',
    'wrist_3_link',
    'flange',
    'tool0',
    'ft_frame',
])

PREFIX = 'arm_'


def _prefix_if_ur(frame_id: str) -> str:
    """Add arm_ prefix if the frame belongs to the UR arm."""
    bare = frame_id.lstrip('/')
    if bare in UR_FRAMES:
        return PREFIX + bare
    return frame_id


class TfPrefixRelay(Node):
    def __init__(self):
        super().__init__('tf_prefix_relay')

        tf_qos = QoSProfile(depth=100)
        tf_static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(TFMessage, '/tf', self._tf_cb, tf_qos)
        self.create_subscription(TFMessage, '/tf_static', self._tf_static_cb, tf_static_qos)

        self._pub = self.create_publisher(TFMessage, '/tf_prefixed', tf_qos)
        self._pub_static = self.create_publisher(
            TFMessage, '/tf_static_prefixed', tf_static_qos)

        self.get_logger().info(
            'TF prefix relay running — UR frames get arm_ prefix on '
            '/tf_prefixed and /tf_static_prefixed')

    def _relay(self, msg: TFMessage) -> TFMessage:
        out = TFMessage()
        for t in msg.transforms:
            # Copy the transform, prefix UR frames
            t.header.frame_id = _prefix_if_ur(t.header.frame_id)
            t.child_frame_id = _prefix_if_ur(t.child_frame_id)
            out.transforms.append(t)
        return out

    def _tf_cb(self, msg: TFMessage):
        self._pub.publish(self._relay(msg))

    def _tf_static_cb(self, msg: TFMessage):
        self._pub_static.publish(self._relay(msg))


def main():
    rclpy.init()
    node = TfPrefixRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
