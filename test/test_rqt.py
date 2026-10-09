"""Shutdown races are quiet; failures with an active ROS context propagate."""

from types import SimpleNamespace

import pytest
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation

from dvrk_simulator_base.rqt import shutdown_safe_run


@pytest.mark.parametrize("error_type", [ExternalShutdownException, rclpy_implementation.RCLError])
@pytest.mark.parametrize("active", [True, False])
def test_spinner_exception_guard(error_type, active):
    context = SimpleNamespace(ok=lambda: active)
    spinner = SimpleNamespace(_node=SimpleNamespace(context=context))

    def run(spinner):
        raise error_type("spinner interrupted")

    guarded = shutdown_safe_run(run)
    if active:
        with pytest.raises(error_type):
            guarded(spinner)
    else:
        assert guarded(spinner) is None


def test_normal_spinner_run_is_preserved():
    spinner = object()
    assert shutdown_safe_run(lambda actual: actual)(spinner) is spinner


def test_unrelated_errors_are_preserved_after_shutdown():
    spinner = SimpleNamespace(_node=SimpleNamespace(context=SimpleNamespace(ok=lambda: False)))

    def run(spinner):
        raise ValueError("invalid plugin data")

    with pytest.raises(ValueError, match="invalid plugin data"):
        shutdown_safe_run(run)(spinner)
