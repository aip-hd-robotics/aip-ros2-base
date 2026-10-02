#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    import yaml
except ImportError:  # pragma: no cover - handled at runtime
    yaml = None


DEFAULT_JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


def load_pose_file(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Pose file does not exist: {path}")

    suffix = path.suffix.lower()
    if suffix == ".json":
        with path.open("r", encoding="utf-8") as file_handle:
            data = json.load(file_handle)
    elif suffix in {".yaml", ".yml"}:
        if yaml is None:
            raise RuntimeError(
                "PyYAML is not installed, so YAML pose files cannot be read."
            )
        with path.open("r", encoding="utf-8") as file_handle:
            data = yaml.safe_load(file_handle)
    else:
        raise ValueError(
            f"Unsupported pose file extension '{path.suffix}'. Use .json, .yaml, or .yml."
        )

    if not isinstance(data, Mapping):
        raise ValueError("Pose file must contain a mapping at the top level.")

    return dict(data)


def pick_preset_container(data: Mapping[str, Any]) -> Mapping[str, Any]:
    for key in ("poses", "presets", "positions"):
        value = data.get(key)
        if isinstance(value, Mapping):
            return value

    return {
        key: value
        for key, value in data.items()
        if key not in {"joint_names", "default_joint_names"}
    }


def normalize_joint_names(
    data: Mapping[str, Any],
    tf_prefix: str,
    cli_joint_names: list[str] | None,
) -> list[str]:
    if cli_joint_names:
        joint_names = cli_joint_names
    else:
        joint_names = data.get("joint_names")
        if not isinstance(joint_names, Sequence) or isinstance(joint_names, (str, bytes)):
            joint_names = DEFAULT_JOINT_NAMES
        else:
            joint_names = list(joint_names)

    if tf_prefix:
        joint_names = [
            name if name.startswith(tf_prefix) else f"{tf_prefix}{name}"
            for name in joint_names
        ]

    return joint_names


def resolve_positions(
    preset_name: str,
    preset_entry: Any,
    joint_names: Sequence[str],
) -> list[float]:
    if isinstance(preset_entry, Mapping):
        for key in ("positions", "joints", "values"):
            candidate = preset_entry.get(key)
            if isinstance(candidate, Sequence) and not isinstance(candidate, (str, bytes)):
                return [float(value) for value in candidate]

        if all(name in preset_entry for name in joint_names):
            return [float(preset_entry[name]) for name in joint_names]

        names = preset_entry.get("joint_names")
        candidate = preset_entry.get("positions") or preset_entry.get("joints")
        if isinstance(names, Sequence) and isinstance(candidate, Sequence):
            ordered_names = list(names)
            if len(ordered_names) != len(candidate):
                raise ValueError(
                    f"Preset '{preset_name}' defines {len(ordered_names)} joint names and {len(candidate)} positions."
                )
            joint_map = {
                str(name): float(value)
                for name, value in zip(ordered_names, candidate, strict=False)
            }
            return [joint_map[name] for name in joint_names]

    if isinstance(preset_entry, Sequence) and not isinstance(
        preset_entry, (str, bytes)
    ):
        return [float(value) for value in preset_entry]

    raise ValueError(
        f"Preset '{preset_name}' must be a list of positions or a mapping with position data."
    )


class NamedPoseSender(Node):
    def __init__(self, action_name: str) -> None:
        super().__init__("go_to_named_position")
        self._action_name = action_name
        self._client = ActionClient(self, FollowJointTrajectory, action_name)

    def send(self, joint_names: Sequence[str], positions: Sequence[float], duration_s: float) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Trajectory action server is not available.")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = Duration(seconds=duration_s).to_msg()
        goal.trajectory.points = [point]

        self.get_logger().info(
            f"Sending {len(point.positions)} joint targets to {self._action_name}"
        )
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error("Trajectory execution did not return a result.")
            return False

        code = result.result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Trajectory execution failed with error code {code}."
            )
            return False

        self.get_logger().info("Trajectory executed successfully.")
        return True


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Send the AR4 robot to a named joint position loaded from a YAML or JSON file."
        )
    )
    parser.add_argument(
        "--file",
        required=True,
        help="Path to a YAML or JSON file that contains named joint targets.",
    )
    parser.add_argument(
        "--name",
        required=False,
        help="Name of the target pose to execute.",
    )
    parser.add_argument(
        "--action-name",
        default="/joint_trajectory_controller/follow_joint_trajectory",
        help="Trajectory action name exposed by the controller.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Time in seconds to reach the target pose.",
    )
    parser.add_argument(
        "--tf-prefix",
        default="",
        help="Optional TF prefix to prepend to the joint names.",
    )
    parser.add_argument(
        "--joint-names",
        nargs="+",
        help=(
            "Optional explicit joint order. If omitted, the script uses joint_1 ... joint_6 "
            "and applies the optional tf prefix."
        ),
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available pose names and exit.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    pose_file = Path(args.file).expanduser().resolve()
    data = load_pose_file(pose_file)
    preset_container = pick_preset_container(data)
    joint_names = normalize_joint_names(data, args.tf_prefix, args.joint_names)

    if args.list:
        print("Available poses:")
        for name in sorted(preset_container.keys()):
            print(f"  {name}")
        return 0

    if not args.name:
        raise ValueError("--name is required unless --list is used.")

    if args.name not in preset_container:
        available = ", ".join(sorted(preset_container.keys())) or "<none>"
        raise KeyError(
            f"Pose '{args.name}' was not found in {pose_file}. Available poses: {available}"
        )

    positions = resolve_positions(args.name, preset_container[args.name], joint_names)

    if len(positions) != len(joint_names):
        raise ValueError(
            f"Preset '{args.name}' provides {len(positions)} values, but {len(joint_names)} joints are expected."
        )

    rclpy.init()
    node = NamedPoseSender(args.action_name)
    try:
        success = node.send(joint_names, positions, args.duration)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())