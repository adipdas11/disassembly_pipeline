"""Shared helpers for EXOTica planner scripts."""

import math
import os
from pathlib import Path
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from .joint_layout import REVOLUTE_JOINTS

XARM_TCP_LINK = "screwdriver_tcp"
UF_TCP_LINK = "rg6_hand_tcp"

XARM_TCP_GOAL = np.array(
    [0.96066, 0.20131, 0.98806, -0.0016332201418885776, -6.310936866669191e-05, -0.000657988951242891],
    dtype=float,
)
UF_TCP_GOAL = np.array(
    [0.9277, 0.035692, 1.1149, -1.6568777222317423, 0.040991142645948074, -0.0030850984424935102],
    dtype=float,
)


def get_config_path(filename: str) -> str:
    """Resolve a config file from this package share directory."""
    pkg_share = Path(get_package_share_directory("dual_arm_exotica"))
    return str(pkg_share / "config" / filename)


def wait_for_joint_state(node, timeout_sec: float = 3.0, joint_names=None) -> np.ndarray:
    """Wait for a full joint state vector in the configured ordering."""
    if joint_names is None:
        raise ValueError("joint_names must be provided for the active EXOTica JointGroup")
    latest = {}

    def callback(msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            latest[name] = position

    sub = node.create_subscription(JointState, "/joint_states", callback, 10)
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(name in latest for name in joint_names):
            node.destroy_subscription(sub)
            return np.array([latest[name] for name in joint_names], dtype=float)

    node.destroy_subscription(sub)
    raise RuntimeError(f"Timed out waiting for /joint_states for joints: {joint_names}")


def wrap_to_nearest(reference: float, angle: float) -> float:
    """Wrap an angle to the nearest branch around a reference angle."""
    return reference + ((angle - reference + math.pi) % (2.0 * math.pi) - math.pi)


def normalize_goal(start_state: np.ndarray, goal_state: np.ndarray, joint_names) -> np.ndarray:
    """Normalize revolute joints relative to the current start state."""
    normalized = np.array(goal_state, dtype=float, copy=True)
    for idx, joint_name in enumerate(joint_names):
        if joint_name in REVOLUTE_JOINTS:
            normalized[idx] = wrap_to_nearest(float(start_state[idx]), float(normalized[idx]))
    return normalized


def unwrap_trajectory(trajectory: np.ndarray, reference: np.ndarray, joint_names) -> np.ndarray:
    """Unwrap a trajectory continuously across revolute joints."""
    unwrapped = np.array(trajectory, dtype=float, copy=True)
    prev = np.array(reference, dtype=float, copy=True)
    for row_idx in range(unwrapped.shape[0]):
        for joint_idx, joint_name in enumerate(joint_names):
            if joint_name in REVOLUTE_JOINTS:
                unwrapped[row_idx, joint_idx] = wrap_to_nearest(prev[joint_idx], unwrapped[row_idx, joint_idx])
        prev = unwrapped[row_idx].copy()
    return unwrapped


def build_two_point_trajectory(joint_names, start_positions, goal_positions, duration: float) -> JointTrajectory:
    """Build a simple two-point trajectory."""
    msg = JointTrajectory()
    msg.joint_names = list(joint_names)

    start_point = JointTrajectoryPoint()
    start_point.positions = [float(p) for p in start_positions]
    start_point.time_from_start = rclpy.duration.Duration(seconds=0.0).to_msg()

    goal_point = JointTrajectoryPoint()
    goal_point.positions = [float(p) for p in goal_positions]
    goal_point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()

    msg.points = [start_point, goal_point]
    return msg


def build_multi_point_trajectory(joint_names, positions: np.ndarray, tau: float) -> JointTrajectory:
    """Build a trajectory from a dense matrix of waypoints."""
    msg = JointTrajectory()
    msg.joint_names = list(joint_names)
    for idx, waypoint in enumerate(positions):
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in waypoint]
        point.time_from_start = rclpy.duration.Duration(seconds=idx * tau).to_msg()
        msg.points.append(point)
    return msg


def interpolate_joint_path(
    start_positions: np.ndarray,
    goal_positions: np.ndarray,
    max_joint_step: float = 0.03,
) -> np.ndarray:
    """Build a dense linear joint-space path between two states."""
    delta = np.asarray(goal_positions, dtype=float) - np.asarray(start_positions, dtype=float)
    max_delta = float(np.max(np.abs(delta))) if delta.size else 0.0
    num_segments = max(1, int(math.ceil(max_delta / max_joint_step)))
    alphas = np.linspace(0.0, 1.0, num_segments + 1, dtype=float)
    return np.asarray(
        [np.asarray(start_positions, dtype=float) + (delta * alpha) for alpha in alphas],
        dtype=float,
    )


def describe_scene_collisions(scene, safe_distance: float = 0.0, limit: int = 5):
    """Return a compact list of self and world colliding pairs for logging."""
    collisions = []
    seen = set()
    try:
        for check_self_collision in (True, False):
            for proxy in scene.get_collision_distance(check_self_collision):
                if proxy.distance <= safe_distance:
                    key = tuple(sorted((proxy.object_1, proxy.object_2)))
                    if key in seen:
                        continue
                    seen.add(key)
                    collisions.append((proxy.object_1, proxy.object_2, proxy.distance))
                    if len(collisions) >= limit:
                        return collisions
    except Exception:
        return []
    return collisions


def is_state_collision_free(scene, state: np.ndarray, safe_distance: float = 0.0) -> bool:
    """Check whether a single state is free of both self and world collisions."""
    scene.update(state)
    return scene.is_state_valid(True, safe_distance) and scene.is_state_valid(False, safe_distance)


def is_trajectory_collision_free(scene, trajectory: np.ndarray, num_subsamples: int = 10) -> bool:
    """Check a trajectory by subsampling each segment."""
    if trajectory.shape[0] == 0:
        return False
    if trajectory.shape[0] == 1:
        return is_state_collision_free(scene, trajectory[0, :])

    for idx in range(1, trajectory.shape[0]):
        q_prev = trajectory[idx - 1, :]
        q_curr = trajectory[idx, :]
        for sample in np.linspace(q_prev, q_curr, num_subsamples):
            if not is_state_collision_free(scene, sample):
                return False
    return True


def safe_process_exit(code: int) -> None:
    """Exit the process without hitting EXOTica teardown crashes."""
    try:
        import sys

        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        os._exit(code)


def compute_pose_error(scene, state: np.ndarray, link_name: str, target_pose: np.ndarray):
    """Compute world-frame position and orientation error for a given link."""
    scene.update(state)
    actual_pose = np.asarray(scene.fk(link_name).get_translation_and_rpy(), dtype=float)
    position_error = float(np.linalg.norm(actual_pose[:3] - target_pose[:3]))
    angle_error = actual_pose[3:] - target_pose[3:]
    angle_error = np.array([math.atan2(math.sin(v), math.cos(v)) for v in angle_error], dtype=float)
    orientation_error = float(np.linalg.norm(angle_error))
    return position_error, orientation_error, actual_pose
