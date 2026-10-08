"""Convert complete simulation scenes into the Cartesian ROS frame contract."""

from dataclasses import replace

from .cartesian_frames import relative_pose, relative_twist, view_pose_from_optical
from .rotations import quaternion_matrix_xyzw
from .snapshots import ArmPublicationFrames
from .types import Pose


def with_publication_frames(snapshots, configs):
    """All arms and the moving ECM reference come from the same completed step."""
    ecm_config = next((config for config in configs if config.type == "ECM"), None)
    ecm = None if ecm_config is None else snapshots[ecm_config.name]
    view = None if ecm is None else view_pose_from_optical(ecm.measured_cp_world)
    result = {}
    for config in configs:
        snapshot = snapshots[config.name]
        measured, setpoint, velocity = snapshot.measured_cp_world, snapshot.setpoint_cp_world, snapshot.measured_cv_world
        frame_id = config.parent_frame
        local_measured = local_setpoint = None
        if config.type == "PSM":
            base = Pose(config.base_position, quaternion_matrix_xyzw(config.base_orientation_xyzw))
            local_measured = relative_pose(measured, base)
            local_setpoint = relative_pose(setpoint, base)
            if view is not None:
                frame_id = "ECM_view"
                measured, setpoint = relative_pose(measured, view), relative_pose(setpoint, view)
                velocity = relative_twist(snapshot.measured_cp_world, velocity, view, ecm.measured_cv_world)
        frames = ArmPublicationFrames(frame_id, measured, setpoint, velocity, local_measured, local_setpoint)
        result[config.name] = replace(snapshot, publication_frames=frames)
    return result
