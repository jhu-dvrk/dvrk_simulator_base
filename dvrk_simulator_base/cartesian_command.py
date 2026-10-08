"""Plain Cartesian commands resolved against simulation-owned frame state."""

from dataclasses import dataclass

from .cartesian_frames import compose_pose, view_pose_from_optical
from .rotations import quaternion_matrix_xyzw
from .types import Pose


@dataclass(frozen=True)
class CartesianCommand:
    pose: Pose
    frame_id: str = ""


def resolve_cartesian_command(command: CartesianCommand, config,
                              ecm_optical_pose: Pose | None = None,
                              *, has_ecm: bool = False) -> Pose:
    """Resolve at consumption time using the last completed simulation state.

    Empty PSM frames retain CRTK's ECM_view default when an ECM is present.
    Explicit world/parent and base frames retain their existing semantics.
    """
    frame = command.frame_id
    if frame == config.base_frame:
        base = Pose(config.base_position, quaternion_matrix_xyzw(config.base_orientation_xyzw))
        return compose_pose(base, command.pose)
    if frame == config.parent_frame:
        return command.pose
    if has_ecm:
        if frame not in ("", "ECM_view"):
            raise ValueError(f"unsupported frame {frame!r}")
        if ecm_optical_pose is None:
            raise ValueError("ECM state is unavailable")
        return compose_pose(view_pose_from_optical(ecm_optical_pose), command.pose)
    if frame:
        raise ValueError(f"unsupported frame {frame!r}")
    return command.pose
