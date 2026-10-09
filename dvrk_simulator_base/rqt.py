"""Launch rqt with a guard for Jazzy's ROS context shutdown race."""

from functools import wraps

from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation


def shutdown_safe_run(run):
    """Ignore spinner shutdown exceptions only after its ROS context closes."""
    @wraps(run)
    def guarded(spinner):
        try:
            return run(spinner)
        except (ExternalShutdownException, rclpy_implementation.RCLError):
            if spinner._node.context.ok():
                raise
    return guarded


def main():
    from rqt_gui.main import Main

    class ShutdownSafeRqt(Main):
        def _add_plugin_providers(self):
            # Let rqt select its Qt binding before importing the spinner.
            from rqt_gui_py.rclpy_spinner import RclpySpinner

            RclpySpinner.run = shutdown_safe_run(RclpySpinner.run)
            super()._add_plugin_providers()

    return ShutdownSafeRqt().main()


if __name__ == "__main__":
    raise SystemExit(main())
