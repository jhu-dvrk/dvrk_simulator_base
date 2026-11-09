from dataclasses import dataclass
import os
from pathlib import Path
import socket

import numpy as np
import pytest

from dvrk_simulator_base.video import (
    UnixFdVideoSink,
    VideoFrame,
    VideoSinkError,
)


@dataclass
class _SimpleCameraOptions:
    socket_path: str | Path
    width: int
    height: int
    rate_hz: float = 30.0
    mode: str = "mono"

    @property
    def transport_width(self) -> int:
        return self.width * (2 if self.mode == "stereo" else 1)


def test_unixfd_sink_accepts_memfd_rgba_and_cleans_socket(tmp_path):
    path = tmp_path / "camera.sock"
    options = _SimpleCameraOptions(socket_path=path, width=16, height=12, rate_hz=30.0)
    sink = UnixFdVideoSink(options)
    try:
        sink.start()
        assert path.is_socket()
        rgba = np.full((12, 16, 4), [10, 20, 30, 255], dtype=np.uint8)
        sink.push(VideoFrame(rgba, 0.0, 0))
        assert sink.frames_pushed == 1
    finally:
        sink.close()
    assert not path.exists()


def test_unixfd_sink_accepts_dvrk_abstract_socket():
    reference = f"@dvrk:simbase:pytest-{os.getpid()}"
    options = _SimpleCameraOptions(socket_path=reference, width=16, height=12)
    sink = UnixFdVideoSink(options)
    try:
        sink.start()
        rgba = np.full((12, 16, 4), [10, 20, 30, 255], dtype=np.uint8)
        sink.push(VideoFrame(rgba, 0.0, 0))
        assert sink.frames_pushed == 1
    finally:
        sink.close()


def test_unixfd_sink_reports_busy_abstract_socket():
    reference = f"@dvrk:simbase:pytest-busy-{os.getpid()}"
    blocker = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    blocker.bind(f"\0{reference[1:]}")
    blocker.listen()
    sink = UnixFdVideoSink(_SimpleCameraOptions(socket_path=reference, width=16, height=12))
    try:
        with pytest.raises(VideoSinkError, match="abstract socket is already in use"):
            sink.start()
    finally:
        sink.close()
        blocker.close()


@pytest.mark.parametrize("mode", ["mono", "stereo"])
@pytest.mark.parametrize("abstract", [False, True])
def test_frame_timestamps_survive_unixfd_transport(tmp_path, mode, abstract):
    reference = (
        f"@dvrk:simbase:pytest-timestamps-{os.getpid()}-{mode}"
        if abstract else tmp_path / "timestamps.sock"
    )
    options = _SimpleCameraOptions(reference, width=16, height=12, mode=mode)
    sink = UnixFdVideoSink(options)
    Gst = sink.Gst
    receiver = Gst.Pipeline.new("timestamp-receiver")
    source = Gst.ElementFactory.make("unixfdsrc", "input")
    appsink = Gst.ElementFactory.make("appsink", "frames")
    source.set_property("socket-path", reference[1:] if abstract else str(reference))
    if abstract:
        source.set_property("socket-type", sink.Gio.UnixSocketAddressType.ABSTRACT)
    appsink.set_property("sync", False)
    receiver.add(source)
    receiver.add(appsink)
    assert source.link(appsink)
    try:
        sink.start()
        receiver.set_state(Gst.State.PLAYING)
        rgba = np.full((12, options.transport_width, 4), [10, 20, 30, 255], dtype=np.uint8)
        frame = VideoFrame(rgba, simulation_time=1.25, sequence=7)
        sample = None
        for _ in range(20):
            sink.push(frame)
            sample = appsink.emit("try-pull-sample", 100 * Gst.MSECOND)
            if sample is not None:
                break
        assert sample is not None, "Unix-FD receiver did not receive a camera frame"
        buffer = sample.get_buffer()
        stereo = mode == "stereo"
        assert sample.get_caps().get_structure(0).get_string("dvrk-video-layout") == (
            "stereo-LR" if stereo else "mono"
        )
        # Both transport pixels and GStreamer's running-time PTS remain intact.
        assert buffer.extract_dup(0, buffer.get_size()) == rgba.tobytes()
        assert buffer.pts != Gst.CLOCK_TIME_NONE
    finally:
        receiver.set_state(Gst.State.NULL)
        receiver.get_state(Gst.SECOND)
        sink.close()
