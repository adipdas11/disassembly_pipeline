"""UF850 EXOTica planner with exact approach pose plus short Cartesian Z descent."""

import time

import numpy as np
import pyexotica as exo
import rclpy
from trajectory_msgs.msg import JointTrajectory

from .joint_layout import UF_PLANNING_JOINTS
from .planner_utils import build_multi_point_trajectory
from .planner_utils import compute_pose_error
from .planner_utils import describe_scene_collisions
from .planner_utils import get_config_path
from .planner_utils import interpolate_joint_path
from .planner_utils import is_state_collision_free
from .planner_utils import is_trajectory_collision_free
from .planner_utils import safe_process_exit
from .planner_utils import UF_TCP_GOAL
from .planner_utils import UF_TCP_LINK
from .planner_utils import unwrap_trajectory
from .planner_utils import wait_for_joint_state


def _declare_params(node) -> dict:
    return {
        "descent_distance": node.declare_parameter("descent_distance", 0.05).value,
        "descent_steps": node.declare_parameter("descent_steps", 25).value,
        "trajectory_dt": node.declare_parameter("trajectory_dt", 0.08).value,
        "max_joint_step": node.declare_parameter("max_joint_step", 0.03).value,
        "position_tolerance": node.declare_parameter("position_tolerance", 0.03).value,
        "orientation_tolerance": node.declare_parameter("orientation_tolerance", 0.15).value,
    }


def _solve_ik_goal(node, solver, problem, scene, q_seed: np.ndarray, goal_pose: np.ndarray) -> np.ndarray:
    problem.start_state = q_seed
    problem.set_goal("UF850_TCP", goal_pose)
    solution = np.array(solver.solve(), dtype=float)
    if solution.size == 0:
        raise RuntimeError("IK returned no solution")
    q_goal = unwrap_trajectory(np.atleast_2d(solution[0]), q_seed, UF_PLANNING_JOINTS)[0]
    if not is_state_collision_free(scene, q_goal):
        raise RuntimeError(
            f"IK solution remains in collision: {describe_scene_collisions(scene)}"
        )
    return q_goal


def main() -> None:
    """Approach a world-frame TCP pose, then descend 5 cm in world Z via repeated IK."""
    rclpy.init(args=None)
    node = rclpy.create_node("dual_arm_exotica_trajectory_planner")
    params = _declare_params(node)
    exit_code = 0

    try:
        uf_pub = node.create_publisher(JointTrajectory, "/uf_controller/joint_trajectory", 10)

        ik_solver = exo.Setup.load_solver(get_config_path("dual_arm_ik.xml"))
        ik_problem = ik_solver.get_problem()
        scene = ik_problem.get_scene()

        q_start = wait_for_joint_state(node, joint_names=UF_PLANNING_JOINTS)
        if not is_state_collision_free(scene, q_start):
            node.get_logger().warning(
                f"Current start state is in collision: {describe_scene_collisions(scene)}"
            )

        approach_pose = np.array(UF_TCP_GOAL, dtype=float, copy=True)
        descent_pose = np.array(approach_pose, dtype=float, copy=True)
        descent_pose[2] -= float(params["descent_distance"])
        node.get_logger().info(
            "Planning UF850 motion: approach target pose, then descend "
            f"{float(params['descent_distance']):.3f} m in world Z with "
            f"{int(params['descent_steps'])} IK steps at {float(params['trajectory_dt']):.3f} s/step"
        )

        q_approach = _solve_ik_goal(node, ik_solver, ik_problem, scene, q_start, approach_pose)
        approach_pos_err, approach_rot_err, _ = compute_pose_error(scene, q_approach, UF_TCP_LINK, approach_pose)
        node.get_logger().info(
            f"Approach IK pose error | uf pos={approach_pos_err:.4f} rot={approach_rot_err:.4f}"
        )
        if (
            approach_pos_err > float(params["position_tolerance"])
            or approach_rot_err > float(params["orientation_tolerance"])
        ):
            raise RuntimeError("Approach IK does not reach the requested TCP pose within tolerance")

        approach_path = interpolate_joint_path(
            q_start,
            q_approach,
            max_joint_step=float(params["max_joint_step"]),
        )
        if not is_trajectory_collision_free(scene, approach_path, num_subsamples=20):
            raise RuntimeError("Approach joint interpolation collides")

        descent_steps = max(1, int(params["descent_steps"]))
        descent_joint_waypoints = [q_approach]
        q_seed = np.array(q_approach, dtype=float, copy=True)
        for step_idx in range(1, descent_steps + 1):
            alpha = step_idx / descent_steps
            waypoint_pose = np.array(approach_pose, dtype=float, copy=True)
            waypoint_pose[2] = approach_pose[2] + ((descent_pose[2] - approach_pose[2]) * alpha)
            q_waypoint = _solve_ik_goal(node, ik_solver, ik_problem, scene, q_seed, waypoint_pose)
            pos_err, rot_err, _ = compute_pose_error(scene, q_waypoint, UF_TCP_LINK, waypoint_pose)
            if (
                pos_err > float(params["position_tolerance"])
                or rot_err > float(params["orientation_tolerance"])
            ):
                raise RuntimeError(
                    f"Descent IK waypoint {step_idx} exceeds tolerance (pos={pos_err:.4f}, rot={rot_err:.4f})"
                )
            segment = interpolate_joint_path(
                q_seed,
                q_waypoint,
                max_joint_step=float(params["max_joint_step"]),
            )
            if not is_trajectory_collision_free(scene, segment, num_subsamples=20):
                raise RuntimeError(f"Descent segment {step_idx} collides")
            descent_joint_waypoints.append(q_waypoint)
            q_seed = np.array(q_waypoint, dtype=float, copy=True)

        final_pose_err_pos, final_pose_err_rot, _ = compute_pose_error(
            scene, descent_joint_waypoints[-1], UF_TCP_LINK, descent_pose
        )
        node.get_logger().info(
            f"Final descent pose error | uf pos={final_pose_err_pos:.4f} rot={final_pose_err_rot:.4f}"
        )

        full_trajectory = [approach_path[0]]
        for waypoint in approach_path[1:]:
            full_trajectory.append(waypoint)
        for idx in range(1, len(descent_joint_waypoints)):
            segment = interpolate_joint_path(
                descent_joint_waypoints[idx - 1],
                descent_joint_waypoints[idx],
                max_joint_step=float(params["max_joint_step"]),
            )
            for waypoint in segment[1:]:
                full_trajectory.append(waypoint)
        full_trajectory = np.asarray(full_trajectory, dtype=float)

        if not is_trajectory_collision_free(scene, full_trajectory, num_subsamples=20):
            raise RuntimeError("Final published trajectory collides")

        uf_pub.publish(
            build_multi_point_trajectory(
                UF_PLANNING_JOINTS,
                full_trajectory,
                float(params["trajectory_dt"]),
            )
        )

        deadline = time.time() + (float(params["trajectory_dt"]) * max(len(full_trajectory) - 1, 1)) + 0.5
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as exc:
        node.get_logger().error(f"EXOTica trajectory planner failed: {exc}")
        exit_code = 1
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        finally:
            safe_process_exit(exit_code)
