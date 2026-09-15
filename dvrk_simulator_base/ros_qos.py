"""QoS profiles shared by CRTK ROS interfaces."""

from __future__ import annotations


def transient_local_event_qos():
    """Reliable retained QoS for operating-state and state events."""
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


def transient_local_latched_qos():
    """Reliable retained QoS for one static value such as tool type."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos
