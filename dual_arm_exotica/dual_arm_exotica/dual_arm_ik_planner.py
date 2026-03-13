"""Standalone EXOTica IK demo entry point for the dual-arm cell."""

import numpy as np
import pyexotica as exo
import rclpy
from trajectory_msgs.msg import JointTrajectory

from .joint_layout import UF_PLANNING_JOINTS
from .planner_utils import build_two_point_trajectory
from .planner_utils import compute_pose_error
from .planner_utils import describe_scene_collisions
from .planner_utils import get_config_path
from .planner_utils import is_state_collision_free
from .planner_utils import safe_process_exit
from .planner_utils import UF_TCP_GOAL
from .planner_utils import UF_TCP_LINK
from .planner_utils import wait_for_joint_state


def main() -> None:
    """Solve a dual-arm EXOTica IK problem and preview it on the controllers."""
    rclpy.init(args=None)
    node = rclpy.create_node("dual_arm_exotica_ik_planner")
    exit_code = 0

    try:
        uf_pub = node.create_publisher(JointTrajectory, "/uf_controller/joint_trajectory", 10)

        solver = exo.Setup.load_solver(get_config_path("dual_arm_ik.xml"))
        problem = solver.get_problem()
        scene = problem.get_scene()

        problem.set_goal("UF850_TCP", UF_TCP_GOAL)

        q_start = wait_for_joint_state(node, joint_names=UF_PLANNING_JOINTS)
        if not is_state_collision_free(scene, q_start):
            node.get_logger().warning(
                f"Current start state is in collision: {describe_scene_collisions(scene)}"
            )

        problem.start_state = q_start
        solution = solver.solve()
        if len(solution) == 0:
            node.get_logger().error("EXOTica IK returned no solution")
            exit_code = 1
        else:
            q_goal = np.array(solution[0], dtype=float)
            if not is_state_collision_free(scene, q_goal):
                node.get_logger().error(
                    f"Rejected IK solution because it remains in collision: {describe_scene_collisions(scene)}"
                )
                exit_code = 1
            else:
                uf_pos_err, uf_rot_err, _ = compute_pose_error(scene, q_goal, UF_TCP_LINK, UF_TCP_GOAL)
                node.get_logger().info(
                    f"IK final pose error | uf pos={uf_pos_err:.4f} rot={uf_rot_err:.4f}"
                )
                uf_pub.publish(build_two_point_trajectory(UF_PLANNING_JOINTS, q_start, q_goal, 2.0))
    except Exception as exc:
        node.get_logger().error(f"EXOTica IK planner failed: {exc}")
        exit_code = 1
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        finally:
            safe_process_exit(exit_code)
