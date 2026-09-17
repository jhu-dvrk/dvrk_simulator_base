import math
import os
import numpy as np
import pytest

os.environ["QT_QPA_PLATFORM"] = "offscreen"

try:
    from PyQt6 import QtWidgets
    from dvrk_simulator_base.arm_widget import SimulatorArmWidget
except (ImportError, SystemExit):
    pytest.skip("PyQt6 is required for SimulatorArmWidget tests", allow_module_level=True)

from dvrk_simulator_base.config import JointConfig, RobotConfig
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.types import JointState, Pose, Twist


@pytest.fixture
def qapp():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


@pytest.fixture
def psm_config():
    joints = (
        JointConfig("yaw", "revolute", -1.5, 1.5, 1.0),
        JointConfig("insertion", "prismatic", 0.0, 0.24, 0.4),
    )
    return RobotConfig(
        name="PSM1",
        type="PSM",
        model="Virtual/PSM1.urdf.xacro",
        instrument="420006",
        endoscope=None,
        parent_frame="world",
        base_frame="PSM1_base",
        rcm_frame="PSM1_RCM",
        tool_frame="PSM1_tool",
        adaptor_frame="PSM1_adaptor",
        base_position=np.zeros(3),
        base_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        joints=joints,
        home_position=np.array([0.0, 0.1]),
        raw={},
    )


def test_arm_widget_creation(qapp, psm_config):
    widget = SimulatorArmWidget(psm_config)

    # 1. Joint table checks: headers have NO (deg) or (mm)
    assert widget.joint_table.columnCount() == 3  # yaw, insertion, jaw
    assert widget.joint_table.horizontalHeaderItem(0).text() == "yaw"
    assert widget.joint_table.horizontalHeaderItem(1).text() == "insertion"
    assert widget.joint_table.horizontalHeaderItem(2).text() == "jaw"
    assert widget.joint_table.rowCount() == 2
    assert widget.joint_table.verticalHeaderItem(0).text() == "Measured"
    assert widget.joint_table.verticalHeaderItem(1).text() == "Command"

    # 2. Cartesian table checks: headers are X, Y, Z, Roll, Pitch, Yaw
    assert widget.cart_table.columnCount() == 6
    assert [widget.cart_table.horizontalHeaderItem(i).text() for i in range(6)] == [
        "X", "Y", "Z", "Roll", "Pitch", "Yaw"
    ]
    assert widget.cart_table.rowCount() == 2
    assert widget.cart_table.verticalHeaderItem(0).text() == "Measured"
    assert widget.cart_table.verticalHeaderItem(1).text() == "Command"


def test_arm_widget_skips_update_when_hidden(qapp, psm_config):
    widget = SimulatorArmWidget(psm_config)
    widget.hide()
    assert not widget.isVisible()

    snapshot = ArmSnapshot(
        sequence=1,
        simulation_time=0.5,
        valid=True,
        measured_js=JointState(("yaw", "insertion"), np.array([0.5, 0.02]), np.zeros(2)),
        setpoint_js=JointState(("yaw", "insertion"), np.zeros(2), np.zeros(2)),
        measured_cp_world=Pose(np.array([0.1, 0.2, 0.3]), np.eye(3)),
        setpoint_cp_world=Pose(np.zeros(3), np.eye(3)),
        measured_cv_world=Twist(np.zeros(3), np.zeros(3)),
        jaw_measured=0.1,
        jaw_setpoint=0.1,
        operating_state=OperatingStateSnapshot("ENABLED", True, False),
    )

    widget.update_snapshot(snapshot)
    # When hidden, table remains unchanged at "0.00"
    assert widget.joint_table.item(0, 0).text() == "0.00"
    assert widget.cart_table.item(0, 0).text() == "0.00"
    assert widget.state_label.text() == "State: UNKNOWN"
    assert widget.homed_label.text() == "Homed"
    assert widget.busy_label.text() == "Busy"
    assert "c62828" in widget.homed_label.styleSheet()
    assert "c62828" in widget.busy_label.styleSheet()


def test_arm_widget_updates_when_visible(qapp, psm_config):
    widget = SimulatorArmWidget(psm_config)
    widget.show()
    qapp.processEvents()

    # Orientation: 90 deg pitch (R_y(pi/2))
    # XYZ convention: pitch = pi/2 -> rotation matrix
    pitch = math.radians(45.0)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rot = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ])
    pos = np.array([0.05, -0.1, 0.2])  # 50 mm, -100 mm, 200 mm

    snapshot = ArmSnapshot(
        sequence=1,
        simulation_time=0.5,
        valid=True,
        measured_js=JointState(("yaw", "insertion"), np.array([math.radians(30.0), 0.05]), np.zeros(2)),
        setpoint_js=JointState(("yaw", "insertion"), np.zeros(2), np.zeros(2)),
        measured_cp_world=Pose(pos, rot),
        setpoint_cp_world=Pose(np.zeros(3), np.eye(3)),
        measured_cv_world=Twist(np.zeros(3), np.zeros(3)),
        jaw_measured=math.radians(15.0),
        jaw_setpoint=math.radians(15.0),
        operating_state=OperatingStateSnapshot("ENABLED", True, True),
    )

    widget.update_snapshot(snapshot)

    assert widget.state_label.text() == "State: ENABLED"
    assert widget.homed_label.text() == "Homed"
    assert widget.busy_label.text() == "Busy"
    assert "2e7d32" in widget.homed_label.styleSheet()
    assert "2e7d32" in widget.busy_label.styleSheet()

    # Joint positions
    assert widget.joint_table.item(0, 0).text() == "30.00"  # deg
    assert widget.joint_table.item(0, 1).text() == "50.00"  # mm
    assert widget.joint_table.item(0, 2).text() == "15.00"  # jaw deg

    # Spinners followed measured because not dirty
    assert widget._joint_spinners[0].value() == 30.00
    assert widget._joint_spinners[1].value() == 50.00
    assert widget._joint_spinners[2].value() == 15.00

    # Cartesian pose: X, Y, Z in mm, R, P, Y in degrees
    assert widget.cart_table.item(0, 0).text() == "50.00"    # X mm
    assert widget.cart_table.item(0, 1).text() == "-100.00"  # Y mm
    assert widget.cart_table.item(0, 2).text() == "200.00"   # Z mm
    assert widget.cart_table.item(0, 3).text() == "0.00"     # Roll deg
    assert widget.cart_table.item(0, 4).text() == "45.00"    # Pitch deg
    assert widget.cart_table.item(0, 5).text() == "0.00"     # Yaw deg


def test_arm_widget_commands_and_dirty_handling(qapp, psm_config):
    joint_cmds = []
    cart_cmds = []
    state_cmds = []

    widget = SimulatorArmWidget(
        psm_config,
        on_state_command=lambda cmd: state_cmds.append(cmd),
        on_joint_command=lambda joints, jaw: joint_cmds.append((joints, jaw)),
        on_cartesian_command=lambda pose: cart_cmds.append(pose),
    )
    widget.show()
    qapp.processEvents()

    # 1. State command test
    home_index = widget.state_combo.findData("home")
    widget.state_combo.setCurrentIndex(home_index)
    widget.state_combo.activated.emit(home_index)
    assert state_cmds == ["home"]

    # 2. Joint command test
    widget._joint_spinners[0].setValue(10.0)  # yaw 10 deg
    widget._joint_spinners[1].setValue(80.0)  # insertion 80 mm
    widget._joint_spinners[2].setValue(20.0)  # jaw 20 deg
    assert widget._joints_dirty

    widget.joint_apply_btn.click()
    assert len(joint_cmds) == 1
    applied_joints, applied_jaw = joint_cmds[0]
    assert np.isclose(applied_joints[0], math.radians(10.0))
    assert np.isclose(applied_joints[1], 0.08)  # 80 mm -> 0.08 m
    assert np.isclose(applied_jaw, math.radians(20.0))
    assert not widget._joints_dirty

    # 3. Cartesian command test
    widget._cart_spinners[0].setValue(150.0)  # X: 150 mm
    widget._cart_spinners[1].setValue(-50.0)  # Y: -50 mm
    widget._cart_spinners[2].setValue(250.0)  # Z: 250 mm
    widget._cart_spinners[3].setValue(0.0)    # Roll: 0 deg
    widget._cart_spinners[4].setValue(30.0)   # Pitch: 30 deg
    widget._cart_spinners[5].setValue(0.0)    # Yaw: 0 deg
    assert widget._cartesian_dirty

    widget.cart_apply_btn.click()
    assert len(cart_cmds) == 1
    pose = cart_cmds[0]
    assert np.allclose(pose.position, [0.15, -0.05, 0.25])
    expected_pitch = math.radians(30.0)
    assert np.isclose(pose.orientation[0, 0], math.cos(expected_pitch))
    assert np.isclose(pose.orientation[0, 2], math.sin(expected_pitch))
    assert not widget._cartesian_dirty
