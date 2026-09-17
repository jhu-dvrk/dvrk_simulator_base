"""Reusable PyQt6 monitor widget for a single dVRK arm."""

from __future__ import annotations

import math
from typing import Callable

import numpy as np

try:
    from PyQt6 import QtCore, QtGui, QtWidgets
except ImportError as error:
    raise SystemExit(
        "error: SimulatorArmWidget requires PyQt6; install it in the dVRK environment"
    ) from error

from .config import JointConfig, RobotConfig
from .snapshots import ArmSnapshot
from .types import Pose


def _rpy_from_matrix(rotation: np.ndarray) -> tuple[float, float, float]:
    """Extract roll, pitch, yaw using the simulator's XYZ convention."""
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(float(-rotation[0, 1]), float(rotation[1, 1]))
    return roll, pitch, yaw


_STYLE_GREEN = (
    "background-color: #2e7d32; color: white; font-weight: bold; "
    "border-radius: 3px; padding: 2px 8px;"
)
_STYLE_RED = (
    "background-color: #c62828; color: white; font-weight: bold; "
    "border-radius: 3px; padding: 2px 8px;"
)


class SimulatorArmWidget(QtWidgets.QWidget):
    """Monitor widget displaying state controls, joint table, and Cartesian pose table."""

    _STATE_COMMANDS = (
        "enable",
        "disable",
        "pause",
        "resume",
        "home",
        "unhome",
        "fault",
        "clear_fault",
    )
    _STATE_COMMAND_PLACEHOLDER = "State command…"

    def __init__(
        self,
        config: RobotConfig,
        on_state_command: Callable[[str], None] | None = None,
        on_joint_command: Callable[[list[float], float | None], None] | None = None,
        on_cartesian_command: Callable[[Pose], None] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.config = config
        self.on_state_command = on_state_command
        self.on_joint_command = on_joint_command
        self.on_cartesian_command = on_cartesian_command
        self.joints = tuple(config.joints)
        self.has_jaw = config.type == "PSM"

        self._refreshing = False
        self._joints_dirty = False
        self._cartesian_dirty = False
        self._latest_snapshot: ArmSnapshot | None = None

        layout = QtWidgets.QVBoxLayout(self)

        # 1. Controls & state status bar
        controls_layout = QtWidgets.QHBoxLayout()
        controls_layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignVCenter)

        self.state_combo = QtWidgets.QComboBox()
        self.state_combo.addItem(self._STATE_COMMAND_PLACEHOLDER, None)
        for command in self._STATE_COMMANDS:
            self.state_combo.addItem(command, command)
        self.state_combo.activated.connect(self._on_combo_activated)
        controls_layout.addWidget(self.state_combo)

        self.state_label = QtWidgets.QLabel("State: UNKNOWN")
        self.homed_label = QtWidgets.QLabel("Homed")
        self.homed_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.homed_label.setFixedHeight(24)
        self.homed_label.setToolTip("Homed status (Green: Homed, Red: Not homed)")
        self.homed_label.setStyleSheet(_STYLE_RED)

        self.busy_label = QtWidgets.QLabel("Busy")
        self.busy_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.busy_label.setFixedHeight(24)
        self.busy_label.setToolTip("Busy status (Green: Busy, Red: Not busy)")
        self.busy_label.setStyleSheet(_STYLE_RED)

        controls_layout.addWidget(self.state_label)
        controls_layout.addWidget(self.homed_label)
        controls_layout.addWidget(self.busy_label)
        controls_layout.addStretch()
        layout.addLayout(controls_layout)

        monospace_font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)

        # 2. Joint positions table
        joints_group = QtWidgets.QGroupBox("Joint positions")
        joints_group.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Preferred,
            QtWidgets.QSizePolicy.Policy.Maximum,
        )
        joints_layout = QtWidgets.QVBoxLayout(joints_group)
        joints_layout.setContentsMargins(6, 4, 6, 6)
        joints_layout.setSpacing(4)

        self.joint_table = QtWidgets.QTableWidget()
        self.joint_table.setRowCount(2)
        self.joint_table.setVerticalHeaderLabels(["Measured", "Command"])

        joint_header_labels: list[str] = [joint.name for joint in self.joints]
        if self.has_jaw:
            joint_header_labels.append("jaw")

        joint_col_count = len(joint_header_labels)
        self.joint_table.setColumnCount(joint_col_count)
        self.joint_table.setHorizontalHeaderLabels(joint_header_labels)

        self._joint_measured_items: list[QtWidgets.QTableWidgetItem] = []
        self._joint_spinners: list[QtWidgets.QDoubleSpinBox] = []

        for col, joint in enumerate(self.joints):
            # Row 0: Measured
            item = QtWidgets.QTableWidgetItem("0.00")
            item.setTextAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            item.setFont(monospace_font)
            item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
            self.joint_table.setItem(0, col, item)
            self._joint_measured_items.append(item)

            # Row 1: Command spinner
            spinner = QtWidgets.QDoubleSpinBox()
            if joint.type == "revolute":
                lower = math.degrees(joint.lower) if np.isfinite(joint.lower) else -360.0
                upper = math.degrees(joint.upper) if np.isfinite(joint.upper) else 360.0
            else:
                lower = joint.lower * 1000.0 if np.isfinite(joint.lower) else -1000.0
                upper = joint.upper * 1000.0 if np.isfinite(joint.upper) else 1000.0
            spinner.setRange(lower, upper)
            spinner.setDecimals(2)
            spinner.setSingleStep(1.0)
            spinner.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            spinner.setFont(monospace_font)
            spinner.valueChanged.connect(lambda _val: self._mark_joints_dirty())
            self.joint_table.setCellWidget(1, col, spinner)
            self._joint_spinners.append(spinner)

        if self.has_jaw:
            # Jaw Measured
            jaw_item = QtWidgets.QTableWidgetItem("0.00")
            jaw_item.setTextAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            jaw_item.setFont(monospace_font)
            jaw_item.setFlags(jaw_item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
            self.joint_table.setItem(0, joint_col_count - 1, jaw_item)
            self._joint_measured_items.append(jaw_item)

            # Jaw Command
            jaw_spinner = QtWidgets.QDoubleSpinBox()
            jaw_spinner.setRange(-30.0, 90.0)
            jaw_spinner.setDecimals(2)
            jaw_spinner.setSingleStep(1.0)
            jaw_spinner.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            jaw_spinner.setFont(monospace_font)
            jaw_spinner.valueChanged.connect(lambda _val: self._mark_joints_dirty())
            self.joint_table.setCellWidget(1, joint_col_count - 1, jaw_spinner)
            self._joint_spinners.append(jaw_spinner)

        self._configure_table_display(self.joint_table)
        joints_layout.addWidget(self.joint_table)

        joints_btn_layout = QtWidgets.QHBoxLayout()
        joints_btn_layout.addStretch()
        self.joint_revert_btn = QtWidgets.QPushButton("Revert")
        self.joint_revert_btn.clicked.connect(self._revert_joints)
        self.joint_apply_btn = QtWidgets.QPushButton("Apply")
        self.joint_apply_btn.clicked.connect(self._apply_joints)
        joints_btn_layout.addWidget(self.joint_revert_btn)
        joints_btn_layout.addWidget(self.joint_apply_btn)
        joints_layout.addLayout(joints_btn_layout)

        layout.addWidget(joints_group)

        # 3. Cartesian pose table
        cart_group = QtWidgets.QGroupBox("Cartesian pose")
        cart_group.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Preferred,
            QtWidgets.QSizePolicy.Policy.Maximum,
        )
        cart_layout = QtWidgets.QVBoxLayout(cart_group)
        cart_layout.setContentsMargins(6, 4, 6, 6)
        cart_layout.setSpacing(4)

        self.cart_table = QtWidgets.QTableWidget()
        self.cart_table.setRowCount(2)
        self.cart_table.setVerticalHeaderLabels(["Measured", "Command"])

        cart_headers = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        self.cart_table.setColumnCount(len(cart_headers))
        self.cart_table.setHorizontalHeaderLabels(cart_headers)

        self._cart_measured_items: list[QtWidgets.QTableWidgetItem] = []
        self._cart_spinners: list[QtWidgets.QDoubleSpinBox] = []

        for col, name in enumerate(cart_headers):
            # Row 0: Measured
            item = QtWidgets.QTableWidgetItem("0.00")
            item.setTextAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            item.setFont(monospace_font)
            item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
            self.cart_table.setItem(0, col, item)
            self._cart_measured_items.append(item)

            # Row 1: Command spinner
            spinner = QtWidgets.QDoubleSpinBox()
            if name in ("X", "Y", "Z"):
                spinner.setRange(-2000.0, 2000.0)
            else:
                spinner.setRange(-180.0, 180.0)
            spinner.setDecimals(2)
            spinner.setSingleStep(1.0)
            spinner.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            spinner.setFont(monospace_font)
            spinner.valueChanged.connect(lambda _val: self._mark_cartesian_dirty())
            self.cart_table.setCellWidget(1, col, spinner)
            self._cart_spinners.append(spinner)

        self._configure_table_display(self.cart_table)
        cart_layout.addWidget(self.cart_table)

        cart_btn_layout = QtWidgets.QHBoxLayout()
        cart_btn_layout.addStretch()
        self.cart_revert_btn = QtWidgets.QPushButton("Revert")
        self.cart_revert_btn.clicked.connect(self._revert_cartesian)
        self.cart_apply_btn = QtWidgets.QPushButton("Apply")
        self.cart_apply_btn.clicked.connect(self._apply_cartesian)
        cart_btn_layout.addWidget(self.cart_revert_btn)
        cart_btn_layout.addWidget(self.cart_apply_btn)
        cart_layout.addLayout(cart_btn_layout)

        layout.addWidget(cart_group)
        layout.addStretch()

    @staticmethod
    def _configure_table_display(table: QtWidgets.QTableWidget) -> None:
        table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        table.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.NoSelection)
        table.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        table.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Fixed)
        table.resizeRowsToContents()
        total_h = (
            table.horizontalHeader().sizeHint().height()
            + table.rowHeight(0)
            + table.rowHeight(1)
            + table.frameWidth() * 2
        )
        table.setFixedHeight(total_h)

    def _mark_joints_dirty(self) -> None:
        if not self._refreshing:
            self._joints_dirty = True

    def _mark_cartesian_dirty(self) -> None:
        if not self._refreshing:
            self._cartesian_dirty = True

    def _apply_joints(self) -> None:
        values = []
        for index, joint in enumerate(self.joints):
            val = self._joint_spinners[index].value()
            values.append(math.radians(val) if joint.type == "revolute" else val / 1000.0)

        jaw_val: float | None = None
        if self.has_jaw and len(self._joint_spinners) > len(self.joints):
            jaw_val = math.radians(self._joint_spinners[-1].value())

        if self.on_joint_command is not None:
            self.on_joint_command(values, jaw_val)
        self._joints_dirty = False

    def _revert_joints(self) -> None:
        self._joints_dirty = False
        if self._latest_snapshot is not None:
            self.update_snapshot(self._latest_snapshot)

    def _apply_cartesian(self) -> None:
        x = self._cart_spinners[0].value() / 1000.0
        y = self._cart_spinners[1].value() / 1000.0
        z = self._cart_spinners[2].value() / 1000.0
        roll = math.radians(self._cart_spinners[3].value())
        pitch = math.radians(self._cart_spinners[4].value())
        yaw = math.radians(self._cart_spinners[5].value())

        position = np.array([x, y, z], dtype=float)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        orientation = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ], dtype=float)

        pose = Pose(position, orientation)
        if self.on_cartesian_command is not None:
            self.on_cartesian_command(pose)
        self._cartesian_dirty = False

    def _revert_cartesian(self) -> None:
        self._cartesian_dirty = False
        if self._latest_snapshot is not None:
            self.update_snapshot(self._latest_snapshot)

    def _on_combo_activated(self, index: int) -> None:
        command = self.state_combo.itemData(index)
        self.state_combo.setCurrentIndex(0)
        if command is not None and self.on_state_command is not None:
            self.on_state_command(command)

    def update_snapshot(self, snapshot: ArmSnapshot) -> None:
        """Update display with snapshot values if the widget is visible."""
        self._latest_snapshot = snapshot
        if not self.isVisible():
            return

        self._refreshing = True
        try:
            self.state_label.setText(f"State: {snapshot.operating_state.state}")
            self.homed_label.setStyleSheet(
                _STYLE_GREEN if snapshot.operating_state.is_homed else _STYLE_RED
            )
            self.busy_label.setStyleSheet(
                _STYLE_GREEN if snapshot.operating_state.is_busy else _STYLE_RED
            )

            # Update Joint positions
            positions = snapshot.measured_js.position
            for index, joint in enumerate(self.joints):
                if index < len(positions):
                    val = float(positions[index])
                    val_converted = math.degrees(val) if joint.type == "revolute" else val * 1000.0
                    self._joint_measured_items[index].setText(f"{val_converted:.2f}")
                    if not self._joints_dirty:
                        self._joint_spinners[index].setValue(val_converted)

            if self.has_jaw and snapshot.jaw_measured is not None:
                jaw_val = math.degrees(float(snapshot.jaw_measured))
                self._joint_measured_items[-1].setText(f"{jaw_val:.2f}")
                if not self._joints_dirty:
                    self._joint_spinners[-1].setValue(jaw_val)

            # Update Cartesian pose
            pose = snapshot.measured_cp_world
            if pose is not None and hasattr(pose, "position") and hasattr(pose, "orientation"):
                x_mm = float(pose.position[0]) * 1000.0
                y_mm = float(pose.position[1]) * 1000.0
                z_mm = float(pose.position[2]) * 1000.0
                roll, pitch, yaw = _rpy_from_matrix(pose.orientation)
                cart_values = [
                    x_mm,
                    y_mm,
                    z_mm,
                    math.degrees(roll),
                    math.degrees(pitch),
                    math.degrees(yaw),
                ]
                for col, val in enumerate(cart_values):
                    self._cart_measured_items[col].setText(f"{val:.2f}")
                    if not self._cartesian_dirty:
                        self._cart_spinners[col].setValue(val)
        finally:
            self._refreshing = False
