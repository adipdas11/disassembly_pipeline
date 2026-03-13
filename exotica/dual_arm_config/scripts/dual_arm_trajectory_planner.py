#!/usr/bin/env python3

from pathlib import Path

import math
import time

import numpy as np
import pyexotica as exo
import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "xarm5_joint1",
    "xarm5_joint2",
    "xarm5_joint3",
    "xarm5_joint4",
    "xarm5_joint5",
    "u1_joint1",
    "u1_joint2",
    "u1_joint3",
    "u1_joint4",
    "u1_joint5",
    "u1_joint6",
    "rg6_l_out",
    "slider_slider_joint",
]

REVOLUTE_JOINTS = {
    "xarm5_joint1",
    "xarm5_joint2",
    "xarm5_joint3",
    "xarm5_joint4",
    "xarm5_joint5",
    "u1_joint1",
    "u1_joint2",
    "u1_joint3",
    "u1_joint4",
    "u1_joint5",
    "u1_joint6",
}


def wait_for_joint_state(node, timeout_sec=3.0):
    latest = {}

    def callback(msg):
        for name, position in zip(msg.name, msg.position):
            latest[name] = position

    sub = node.create_subscription(JointState, "/joint_states", callback, 10)
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(name in latest for name in JOINT_NAMES):
            node.destroy_subscription(sub)
            return np.array([latest[name] for name in JOINT_NAMES], dtype=float)

    node.destroy_subscription(sub)
    raise RuntimeError("Timed out waiting for /joint_states")


def wrap_to_nearest(reference, angle):
    return reference + ((angle - reference + math.pi) % (2.0 * math.pi) - math.pi)


def unwrap_trajectory(trajectory, reference):
    unwrapped = np.array(trajectory, dtype=float, copy=True)
    prev = np.array(reference, dtype=float, copy=True)

    for row_idx in range(unwrapped.shape[0]):
        for joint_idx, joint_name in enumerate(JOINT_NAMES):
            if joint_name in REVOLUTE_JOINTS:
                unwrapped[row_idx, joint_idx] = wrap_to_nearest(prev[joint_idx], unwrapped[row_idx, joint_idx])
        prev = unwrapped[row_idx].copy()

    return unwrapped


def build_trajectory(names, positions, tau):
    msg = JointTrajectory()
    msg.joint_names = names

    for idx, waypoint in enumerate(positions):
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in waypoint]
        point.time_from_start = rclpy.duration.Duration(seconds=idx * tau).to_msg()
        msg.points.append(point)

    return msg


def main():
    try:
        exo.Setup.init_ros()
    except RuntimeError as exc:
        print(f"Skipping EXOTica ROS init: {exc}")

    rclpy.init(args=None)
    node = rclpy.create_node("dual_arm_exotica_trajectory_planner")

    xarm_pub = node.create_publisher(JointTrajectory, "/xarm_controller/joint_trajectory", 10)
    uf_pub = node.create_publisher(JointTrajectory, "/uf_controller/joint_trajectory", 10)
    rg6_pub = node.create_publisher(JointTrajectory, "/rg6_controller/joint_trajectory", 10)
    slider_pub = node.create_publisher(JointTrajectory, "/slider_controller/joint_trajectory", 10)

    xml_path = Path(__file__).resolve().parents[1] / "resources" / "dual_arm_aico.xml"
    print(f"Loading EXOTica trajectory solver from: {xml_path}")

    solver = exo.Setup.load_solver(str(xml_path))
    problem = solver.get_problem()

    q_start = wait_for_joint_state(node)
    problem.start_state = q_start

    uf_goal = np.array([0.4, 0.2, 0.6, 0.0, 0.0, 0.0], dtype=float)
    xarm_goal = np.array([0.4, -0.2, 0.4, 0.0, 1.57, 0.0], dtype=float)

    warmup_steps = max(5, problem.T // 8)
    for t in range(problem.T):
        if t < warmup_steps:
            problem.set_rho("UF850_TCP", 0.0, t)
            problem.set_rho("xArm5_TCP", 0.0, t)
        else:
            problem.set_rho("UF850_TCP", 5e2, t)
            problem.set_rho("xArm5_TCP", 5e2, t)
            problem.set_goal("UF850_TCP", uf_goal, t)
            problem.set_goal("xArm5_TCP", xarm_goal, t)

    print("Solving collision-aware trajectory...")
    solution = np.array(solver.solve(), dtype=float)
    if solution.size == 0:
        print("No trajectory returned.")
        node.destroy_node()
        rclpy.shutdown()
        return

    solution = unwrap_trajectory(solution, q_start)
    print(f"Trajectory solved with {solution.shape[0]} waypoints over {problem.T * problem.tau:.2f}s")

    xarm_pub.publish(build_trajectory(JOINT_NAMES[0:5], solution[:, 0:5], problem.tau))
    uf_pub.publish(build_trajectory(JOINT_NAMES[5:11], solution[:, 5:11], problem.tau))
    rg6_pub.publish(build_trajectory([JOINT_NAMES[11]], solution[:, 11:12], problem.tau))
    slider_pub.publish(build_trajectory([JOINT_NAMES[12]], solution[:, 12:13], problem.tau))

    end_time = time.time() + problem.T * problem.tau + 0.5
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
