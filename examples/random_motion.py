#!/usr/bin/env python3
"""Continuously send small random joint targets to FAIRINO SimMachine."""

from __future__ import annotations

import argparse
import fcntl
import ipaddress
import random
import sys
import time
from pathlib import Path


SDK_ROOT = Path(__file__).resolve().parent / "fairino-python-sdk" / "linux"
LOCK_PATH = Path(__file__).resolve().parent / ".random_motion.lock"
sys.path.insert(0, str(SDK_ROOT))

from fairino import Robot  # noqa: E402


# Maximum offset from the pose captured at startup, in degrees.
JOINT_AMPLITUDES = (18.0, 10.0, 10.0, 15.0, 10.0, 20.0)


def require_success(operation: str, result: int) -> None:
    if result != 0:
        raise RuntimeError(f"{operation} failed with FAIRINO error code {result}")


def get_joint_positions(robot: Robot.RPC) -> list[float]:
    """Read joints directly over XML-RPC, avoiding the SDK's state-thread race."""
    response = robot.robot.GetActualJointPosDegree(0)
    if not isinstance(response, (list, tuple)) or len(response) < 7:
        raise RuntimeError(f"unexpected joint-position response: {response!r}")
    require_success("GetActualJointPosDegree", int(response[0]))
    return [float(value) for value in response[1:7]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move the simulated FAIRINO arm to a random joint target every 2 seconds."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--velocity", type=float, default=20.0)
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="Number of targets; 0 runs until Ctrl+C (default: 0)",
    )
    args = parser.parse_args()

    try:
        loopback = ipaddress.ip_address(args.host).is_loopback
    except ValueError:
        loopback = args.host == "localhost"
    if not loopback:
        parser.error("this random-motion program is restricted to the local simulator")
    if not 0.5 <= args.interval <= 30.0:
        parser.error("--interval must be between 0.5 and 30 seconds")
    if not 1.0 <= args.velocity <= 30.0:
        parser.error("--velocity must be between 1 and 30")
    if args.count < 0:
        parser.error("--count cannot be negative")
    return args


def main() -> int:
    args = parse_args()
    robot = None
    move_number = 0
    lock_file = LOCK_PATH.open("w")

    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(
            "ERROR: another random_motion.py process is already running; stop it first.",
            file=sys.stderr,
        )
        lock_file.close()
        return 2

    try:
        print(f"Connecting to FAIRINO SimMachine at {args.host} ...", flush=True)
        robot = Robot.RPC(args.host)
        require_success("Mode(automatic)", robot.Mode(0))
        require_success("RobotEnable", robot.RobotEnable(1))
        time.sleep(1.0)

        start_joints = get_joint_positions(robot)
        print(
            "Center pose [deg]:",
            [round(value, 2) for value in start_joints],
            flush=True,
        )
        print("Random motion started. Press Ctrl+C to stop.", flush=True)

        while args.count == 0 or move_number < args.count:
            cycle_started = time.monotonic()
            target = [
                center + random.uniform(-amplitude, amplitude)
                for center, amplitude in zip(start_joints, JOINT_AMPLITUDES)
            ]
            move_number += 1
            require_success(
                f"MoveJ(target {move_number})",
                robot.MoveJ(
                    target,
                    tool=0,
                    user=0,
                    vel=args.velocity,
                    acc=0.0,
                    ovl=30.0,
                    blendT=500.0,
                ),
            )
            print(
                f"#{move_number}: {[round(value, 2) for value in target]}",
                flush=True,
            )
            remaining = args.interval - (time.monotonic() - cycle_started)
            if remaining > 0:
                time.sleep(remaining)

        require_success(
            "MoveJ(center)",
            robot.MoveJ(
                start_joints,
                tool=0,
                user=0,
                vel=args.velocity,
                acc=0.0,
                ovl=30.0,
                blendT=-1.0,
            ),
        )
        print("Completed; returned to the center pose.", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\nStopping random motion ...", flush=True)
        if robot is not None:
            robot.StopMotion()
        return 0
    except (ConnectionError, OSError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if robot is not None:
            robot.CloseRPC()
        lock_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
