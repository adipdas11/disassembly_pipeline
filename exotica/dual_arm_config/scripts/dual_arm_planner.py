#!/usr/bin/env python3

from pathlib import Path

import pyexotica as exo
from numpy import array
import math
import time
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


def wrap_to_nearest(reference, angle):
    return reference + ((angle - reference + math.pi) % (2.0 * math.pi) - math.pi)


def normalize_goal(q_start, q_goal):
    q_goal = array(q_goal, dtype=float)
    for idx, joint_name in enumerate(JOINT_NAMES):
        if joint_name in REVOLUTE_JOINTS:
            q_goal[idx] = wrap_to_nearest(float(q_start[idx]), float(q_goal[idx]))
    return q_goal


def build_trajectory(names, start_positions, goal_positions, duration):
    msg = JointTrajectory()
    msg.joint_names = names

    start_point = JointTrajectoryPoint()
    start_point.positions = [float(p) for p in start_positions]
    start_point.time_from_start = rclpy.duration.Duration(seconds=0.0).to_msg()

    goal_point = JointTrajectoryPoint()
    goal_point.positions = [float(p) for p in goal_positions]
    goal_point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()

    msg.points = [start_point, goal_point]
    return msg


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
            sub.destroy()
            return array([latest[name] for name in JOINT_NAMES], dtype=float)

    sub.destroy()
    raise RuntimeError("Timed out waiting for /joint_states from the fake hardware stack")


def main():
    # ROS init is optional for this standalone solver script.
    try:
        exo.Setup.init_ros()
    except RuntimeError as exc:
        print(f"Skipping EXOTica ROS init: {exc}")

    rclpy.init(args=None)
    node = rclpy.create_node("dual_arm_exotica_visualizer")
    xarm_pub = node.create_publisher(JointTrajectory, "/xarm_controller/joint_trajectory", 10)
    uf_pub = node.create_publisher(JointTrajectory, "/uf_controller/joint_trajectory", 10)
    rg6_pub = node.create_publisher(JointTrajectory, "/rg6_controller/joint_trajectory", 10)
    slider_pub = node.create_publisher(JointTrajectory, "/slider_controller/joint_trajectory", 10)

    xml_path = Path(__file__).resolve().parents[1] / "resources" / "dual_arm_ik.xml"
    
    print(f"Loading EXOTica solver from: {xml_path}")
    solver = exo.Setup.load_solver(str(xml_path))
    problem = solver.get_problem()
    
    # 2. Define Goals
    # Format for EffFrame is [x, y, z, roll, pitch, yaw] or similar depending on TaskMap
    # Let's set some reachable positions relative to the robot base
    
    # UF850 Goal: Position in front and slightly high
    uf_goal = array([0.4, 0.2, 0.6, 0.0, 0.0, 0.0])
    
    # xArm5 Goal: Position in front and slightly low
    xarm_goal = array([0.4, -0.2, 0.4, 0.0, 1.57, 0.0]) 
    
    print("Setting goals and solving...")
    problem.set_goal('UF850_TCP', uf_goal)
    problem.set_goal('xArm5_TCP', xarm_goal)
    
    # 3. Solve
    start_time = time.time()
    solution = solver.solve()
    end_time = time.time()
    
    if len(solution) > 0:
        q_start = wait_for_joint_state(node)
        q_sol = normalize_goal(q_start, solution[0])
        print(f"✅ Solve Successful! Time taken: {end_time - start_time:.4f} seconds")
        print("--- Joint Solution ---")
        # Ordering based on SRDF dual_arms group
        # xarm5_joint1-5, u1_joint1-6, rg6_l_out, slider_slider_joint
        print(f"xArm5 JNT: {q_sol[0:5]}")
        print(f"UF850 JNT: {q_sol[5:11]}")
        print(f"Gripper:   {q_sol[11]}")
        print(f"Slider:    {q_sol[12]}")
        print("Sending a 2-second trajectory to the fake controllers for RViz preview...")

        xarm_pub.publish(build_trajectory(JOINT_NAMES[0:5], q_start[0:5], q_sol[0:5], 2.0))
        uf_pub.publish(build_trajectory(JOINT_NAMES[5:11], q_start[5:11], q_sol[5:11], 2.0))
        rg6_pub.publish(build_trajectory([JOINT_NAMES[11]], [q_start[11]], [q_sol[11]], 2.0))
        slider_pub.publish(build_trajectory([JOINT_NAMES[12]], [q_start[12]], [q_sol[12]], 2.0))

        # Keep the node alive long enough for the controller trajectory to finish.
        end_deadline = time.time() + 2.5
        while time.time() < end_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    else:
        print("❌ Solve Failed.")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
