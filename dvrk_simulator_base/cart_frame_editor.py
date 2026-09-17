"""Small optional PyQt6 editor for the patient-cart RCM sphere layout."""

from __future__ import annotations


def main(arguments: list[str] | None = None) -> None:
    """Launch the editor without importing any simulator backend."""
    try:
        from PyQt6 import QtCore, QtGui, QtWidgets
    except ImportError as error:
        raise SystemExit(
            "error: cart_frame_editor requires PyQt6; install it in the dVRK virtual environment"
        ) from error

    import argparse
    from pathlib import Path
    import signal

    from .cart_frames import (
        DEFAULT_SPHERICAL_COORDINATES,
        SPHERE_RADIUS_M,
        load_scene_coordinates,
        rcm_position,
        update_scene_file,
        yaml_frames,
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-s", "--scene", metavar="SCENE.yaml", help="load and save this scene YAML")
    options = parser.parse_args(arguments)
    scene_path = None if options.scene is None else Path(options.scene).expanduser().resolve()
    try:
        initial_coordinates = (
            DEFAULT_SPHERICAL_COORDINATES
            if scene_path is None
            else load_scene_coordinates(scene_path)
        )
    except (FileNotFoundError, ValueError) as error:
        parser.error(str(error))

    colors = {"PSM1": "#2878b5", "PSM2": "#d65f36", "PSM3": "#4d9b55", "ECM": "#8c5db3"}

    class LayoutPreview(QtWidgets.QWidget):
        def __init__(self, coordinates, parent=None) -> None:
            super().__init__(parent)
            self.coordinates = coordinates
            self.setMinimumSize(680, 330)

        def paintEvent(self, event) -> None:
            painter = QtGui.QPainter(self)
            painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
            painter.fillRect(self.rect(), QtGui.QColor("white"))
            self._draw_projection(painter, "Top: X / Y", 170, 165, 0, 1)
            self._draw_projection(painter, "Side: Y / Z", 510, 165, 1, 2)

        def _draw_projection(self, painter, title, center_x, center_y, horizontal, vertical) -> None:
            scale = 130 / SPHERE_RADIUS_M
            painter.setPen(QtGui.QPen(QtGui.QColor("#202124")))
            painter.drawText(center_x - 70, 22, title)
            painter.setPen(QtGui.QPen(QtGui.QColor("#9aa0a6")))
            painter.drawEllipse(center_x - 130, center_y - 130, 260, 260)
            painter.setPen(QtGui.QPen(QtGui.QColor("#d7dce0")))
            painter.drawLine(center_x - 145, center_y, center_x + 145, center_y)
            painter.drawLine(center_x, center_y - 145, center_x, center_y + 145)
            for name, (azimuth, polar) in self.coordinates().items():
                position = rcm_position(azimuth, polar)
                x = center_x + position[horizontal] * scale
                y = center_y - position[vertical] * scale
                color = QtGui.QColor(colors[name])
                painter.setPen(QtGui.QPen(color, 2))
                painter.drawLine(QtCore.QPointF(x, y), QtCore.QPointF(center_x, center_y))
                painter.setBrush(QtGui.QBrush(color))
                painter.setPen(QtGui.QPen(QtGui.QColor("white"), 1))
                painter.drawEllipse(QtCore.QPointF(x, y), 5, 5)
                painter.setPen(QtGui.QPen(color))
                painter.drawText(QtCore.QPointF(x + 8, y - 8), name)

    class CartFrameEditor(QtWidgets.QWidget):
        def __init__(self, coordinates, source_scene) -> None:
            super().__init__()
            self.setWindowTitle("dVRK simulator cart-frame editor")
            self.scene_path = source_scene
            self.inputs = {}
            layout = QtWidgets.QGridLayout(self)
            controls = QtWidgets.QGridLayout()
            controls.addWidget(QtWidgets.QLabel("Arm"), 0, 0)
            controls.addWidget(QtWidgets.QLabel("Azimuth (°)"), 0, 1)
            controls.addWidget(QtWidgets.QLabel("Polar (°)"), 0, 2)
            for row, (name, (azimuth, polar)) in enumerate(coordinates.items(), start=1):
                controls.addWidget(QtWidgets.QLabel(name), row, 0)
                azimuth_input = self._spin(-180.0, 180.0, azimuth)
                polar_input = self._spin(0.0, 180.0, polar)
                controls.addWidget(azimuth_input, row, 1)
                controls.addWidget(polar_input, row, 2)
                self.inputs[name] = (azimuth_input, polar_input)
            reset = QtWidgets.QPushButton("Reset defaults")
            reset.clicked.connect(self.reset)
            controls.addWidget(reset, 5, 0, 1, 3)
            note = QtWidgets.QLabel("Azimuth is measured +X toward +Y; polar is down from +Z.\nWorld −Y is the patient-cart front.")
            note.setWordWrap(True)
            controls.addWidget(note, 6, 0, 1, 3)
            layout.addLayout(controls, 0, 0, QtCore.Qt.AlignmentFlag.AlignTop)
            self.preview = LayoutPreview(self.values)
            layout.addWidget(self.preview, 0, 1)
            self.output = QtWidgets.QPlainTextEdit()
            self.output.setReadOnly(True)
            self.output.setMinimumHeight(190)
            layout.addWidget(self.output, 1, 0, 1, 2)
            copy = QtWidgets.QPushButton("Copy YAML")
            copy.clicked.connect(self.copy_yaml)
            save = QtWidgets.QPushButton("Save")
            save.clicked.connect(self.save)
            quit_button = QtWidgets.QPushButton("Quit")
            quit_button.clicked.connect(application.quit)
            buttons = QtWidgets.QHBoxLayout()
            buttons.addWidget(copy)
            buttons.addWidget(save)
            buttons.addWidget(quit_button)
            layout.addLayout(buttons, 2, 1, QtCore.Qt.AlignmentFlag.AlignRight)
            self.status = QtWidgets.QLabel(
                "Save selects a scene YAML" if self.scene_path is None else f"Scene: {self.scene_path}"
            )
            layout.addWidget(self.status, 2, 0)
            self.shortcut = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+C"), self)
            self.shortcut.activated.connect(application.quit)
            self.refresh()

        def _spin(self, minimum, maximum, value):
            spinner = QtWidgets.QDoubleSpinBox()
            spinner.setRange(minimum, maximum)
            spinner.setDecimals(1)
            spinner.setSingleStep(1.0)
            spinner.setValue(value)
            spinner.valueChanged.connect(self.refresh)
            return spinner

        def values(self):
            return {
                name: (azimuth.value(), polar.value())
                for name, (azimuth, polar) in self.inputs.items()
            }

        def refresh(self, *_unused) -> None:
            self.output.setPlainText(yaml_frames(self.values()))
            self.preview.update()

        def reset(self) -> None:
            for name, (azimuth, polar) in DEFAULT_SPHERICAL_COORDINATES.items():
                self.inputs[name][0].setValue(azimuth)
                self.inputs[name][1].setValue(polar)

        def copy_yaml(self) -> None:
            QtWidgets.QApplication.clipboard().setText(self.output.toPlainText())

        def save(self) -> None:
            if self.scene_path is None:
                selected, _ = QtWidgets.QFileDialog.getOpenFileName(
                    self, "Select scene YAML to update", "", "YAML files (*.yaml *.yml)"
                )
                if not selected:
                    return
                self.scene_path = Path(selected)
            try:
                update_scene_file(self.scene_path, self.values())
            except (FileNotFoundError, ValueError, OSError) as error:
                QtWidgets.QMessageBox.critical(self, "Could not save scene", str(error))
                return
            self.status.setText(f"Saved RCM frames to {self.scene_path}")

    application = QtWidgets.QApplication([])
    editor = CartFrameEditor(initial_coordinates, scene_path)
    editor.show()
    # Qt's event loop otherwise delays Python signal delivery.  The timer
    # gives Ctrl+C in the launching ROS 2 terminal a predictable shutdown.
    signal.signal(signal.SIGINT, lambda *_: application.quit())
    signal_timer = QtCore.QTimer(application)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(100)
    application.exec()
