#!/usr/bin/env python3
"""Move the FAIRINO SimMachine robot and optionally return to the start pose."""

from __future__ import annotations

import argparse
import ipaddress
import sys
import time
from pathlib import Path


SDK_ROOT = Path(__file__).resolve().parent / "fairino-python-sdk" / "linux"
sys.path.insert(0, str(SDK_ROOT))

from fairino import Robot  # noqa: E402


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
        description="Move joint 1 of a FAIRINO simulator by a small angle."
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Controller address (default: localhost port-forward to SimMachine)",
    )
    parser.add_argument(
        "--degrees",
        type=float,
        default=10.0,
        help="Joint 1 relative movement in degrees (default: 10)",
    )
    parser.add_argument(
        "--velocity",
        type=float,
        default=10.0,
        help="Movement velocity percentage, 1-30 (default: 10)",
    )
    parser.add_argument(
        "--stay",
        action="store_true",
        help="Do not return to the starting joint pose after the demonstration",
    )
    parser.add_argument(
        "--allow-real-robot",
        action="store_true",
        help="Allow a non-loopback controller address (use only after a safety review)",
    )
    args = parser.parse_args()

    if not -20.0 <= args.degrees <= 20.0 or args.degrees == 0:
        parser.error("--degrees must be non-zero and between -20 and 20")
    if not 1.0 <= args.velocity <= 30.0:
        parser.error("--velocity must be between 1 and 30")

    try:
        is_loopback = ipaddress.ip_address(args.host).is_loopback
    except ValueError:
        is_loopback = args.host == "localhost"
    if not is_loopback and not args.allow_real_robot:
        parser.error(
            "non-loopback hosts require --allow-real-robot; this demo defaults to simulation"
        )
    return args


def main() -> int:
    args = parse_args()
    robot = None

    try:
        print(f"Connecting to FAIRINO controller at {args.host} ...")
        robot = Robot.RPC(args.host)

        require_success("Mode(automatic)", robot.Mode(0))
        require_success("RobotEnable", robot.RobotEnable(1))
        time.sleep(1.0)

        start_joints = get_joint_positions(robot)
        print("Start joints [deg]:", [round(value, 3) for value in start_joints])

        target_joints = start_joints.copy()
        target_joints[0] += args.degrees
        print("Target joints [deg]:", [round(value, 3) for value in target_joints])

        require_success(
            "MoveJ(target)",
            robot.MoveJ(
                target_joints,
                tool=0,
                user=0,
                vel=args.velocity,
                acc=0.0,
                ovl=30.0,
                blendT=-1.0,
            ),
        )
        print("Target reached.")

        if not args.stay:
            time.sleep(1.0)
            require_success(
                "MoveJ(start)",
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
            print("Returned to the starting pose.")

        final_joints = get_joint_positions(robot)
        print("Final joints [deg]:", [round(value, 3) for value in final_joints])
        return 0
    except (ConnectionError, OSError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if robot is not None:
            robot.CloseRPC()


if __name__ == "__main__":
    raise SystemExit(main())
