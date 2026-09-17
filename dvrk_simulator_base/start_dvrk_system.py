#!/usr/bin/env python3

# Author: Anton Deguet
# (C) Copyright 2026 Johns Hopkins University (JHU), All Rights Reserved.

"""Home dVRK system and enable teleoperation on console."""

from __future__ import annotations

import argparse
import sys
import time

import crtk
import dvrk


def main(arguments: list[str] | None = None) -> int:
    if arguments is None:
        arguments = sys.argv[1:]

    # Strip any ROS remapping arguments
    filtered_args = crtk.ral.parse_argv(arguments)

    parser = argparse.ArgumentParser(
        description="Home dVRK system and enable teleoperation on console."
    )
    parser.add_argument(
        "-c",
        "--console",
        type=str,
        default="console",
        help="dVRK console ROS namespace (default: %(default)s)",
    )
    parser.add_argument(
        "-s",
        "--system",
        type=str,
        default="system",
        help="dVRK system ROS namespace (default: %(default)s)",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=float,
        default=30.0,
        help="Timeout in seconds waiting for ROS topic connections (default: %(default)s)",
    )
    parser.add_argument(
        "-w",
        "--wait-time",
        type=float,
        default=3.0,
        help="Time in seconds to wait between homing and teleoperation (default: %(default)s)",
    )
    parser.add_argument(
        "--no-teleop",
        action="store_true",
        help="Only home the system, do not enable teleoperation",
    )
    options = parser.parse_args(filtered_args)

    console_name = options.console
    system_name = options.system

    ral = crtk.ral("start_dvrk_system")
    sys_obj = dvrk.system(ral, system_name)
    con_obj = dvrk.console(ral, console_name)

    def run():
        print(
            f"[start_dvrk_system] Waiting for '{system_name}' and '{console_name}' topic connections..."
        )
        ral.check_connections(timeout_seconds=options.timeout)

        print(f"[start_dvrk_system] Homing system '{system_name}'...")
        sys_obj.home()

        if not options.no_teleop:
            print(f"[start_dvrk_system] Waiting {options.wait_time}s for arms to home...")
            time.sleep(options.wait_time)

            print(f"[start_dvrk_system] Enabling teleoperation on '{console_name}'...")
            con_obj.teleop_start()

        # Brief pause to ensure messages are delivered before shutdown
        time.sleep(1.0)
        print("[start_dvrk_system] Startup completed successfully.")

    try:
        ral.spin_and_execute(run)
        return 0
    except Exception as e:
        print(f"[start_dvrk_system] Error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
