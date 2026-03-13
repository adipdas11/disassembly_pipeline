#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

import math
import threading
import time

# ==========================================
# HELPER FUNCTIONS (Defined at Global Scope)
# ==========================================
def rpy_to_quaternion(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return Quaternion(w=cr*cp*cy + sr*sp*sy, x=sr*cp*cy - cr*sp*sy, y=cr*sp*cy + sr*cp*sy, z=cr*cp*sy - sr*sp*cy)

class UniversalCommander(Node):
    def __init__(self):
        super().__init__('universal_commander')
        self.cb_group = ReentrantCallbackGroup()
        
        # 1. Action Client for Motion
        self._action_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        
        # 2. Service Client for IK
        self._ik_client = self.create_client(GetPositionIK, 'compute_ik', callback_group=self.cb_group)
        
        # 3. State Awareness (Seeding Logic for IK stability)
        self.current_joint_state = None
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_cb, 
            10, 
            callback_group=self.cb_group
        )
        
        self.motion_done_event = threading.Event()
        self.last_result = None
        self.get_logger().info("🚀 Universal Commander V8: Multi-Robot Logic Active")

    def joint_cb(self, msg):
        self.current_joint_state = msg

    def _call_ik_sync(self, req):
        future = self._ik_client.call_async(req)
        while not future.done():
            time.sleep(0.01)
        return future.result()

    def get_ik_solution(self, group_name, target_dict):
        """ 
        Specialized Solver:
        - UF850: Uses Strict Quaternion from your image.
        - xArm5: Uses Relaxation (RPY) to ensure 5-DOF success.
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.ik_link_name = target_dict['link']
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 2 
        
        # Smart Seeding from current joints
        req.ik_request.robot_state = RobotState()
        if self.current_joint_state:
            req.ik_request.robot_state.joint_state = self.current_joint_state
        req.ik_request.robot_state.is_diff = True 
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world_world" 
        target_pose.pose.position.x = target_dict['x']
        target_pose.pose.position.y = target_dict['y']
        target_pose.pose.position.z = target_dict['z']

        # --- BRANCH: UF850 (6-DOF) ---
        if "u1" in target_dict['link']:
            target_pose.pose.orientation.x = target_dict['qx']
            target_pose.pose.orientation.y = target_dict['qy']
            target_pose.pose.orientation.z = target_dict['qz']
            target_pose.pose.orientation.w = target_dict['qw']
            req.ik_request.pose_stamped = target_pose
            res = self._call_ik_sync(req)
            if res and res.error_code.val == 1: return res.solution.joint_state

        # --- BRANCH: xArm5 (5-DOF) or UF Fallback ---
        else:
            # Level 1: Strict Vertical
            target_pose.pose.orientation = rpy_to_quaternion(3.14159, 0.0, 0.0)
            req.ik_request.pose_stamped = target_pose
            res = self._call_ik_sync(req)
            if res and res.error_code.val == 1: return res.solution.joint_state

            # Level 2: Yaw Relaxation
            self.get_logger().warn(f"[{group_name}] Falling back to relaxation...")
            for yaw in [0.2, -0.2, 0.5]:
                req.ik_request.pose_stamped.pose.orientation = rpy_to_quaternion(3.14159, 0.0, yaw)
                res = self._call_ik_sync(req)
                if res and res.error_code.val == 1: return res.solution.joint_state

        self.get_logger().error(f"[{group_name}] IK Failed definitively.")
        return None

    def create_constraints_from_joints(self, joint_state, filter_prefix):
        constraints = Constraints()
        for name, pos in zip(joint_state.name, joint_state.position):
            if filter_prefix in name:
                jc = JointConstraint()
                jc.joint_name, jc.position, jc.weight = name, pos, 1.0
                jc.tolerance_above = jc.tolerance_below = 0.02
                constraints.joint_constraints.append(jc)
        return constraints

    def execute_joint_trajectory(self, group_name, constraints):
        self.motion_done_event.clear()
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 50 
        goal_msg.request.allowed_planning_time = 20.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(f"Moving {group_name}...")
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def move_dual_arms_sequentially(self, x_goal, u_goal):
        """ Moves xArm5 then UF850 to avoid workspace collision issues. """
        self.get_logger().info("--- Starting Sequential Move ---")
        
        # --- PHASE 1: xArm5 ---
        x_js = self.get_ik_solution("xarm_arm", x_goal)
        if x_js:
            self.execute_joint_trajectory("xarm_arm", self.create_constraints_from_joints(x_js, "xarm5"))
            if not self.motion_done_event.wait(timeout=25.0): return

        # --- PHASE 2: UF850 ---
        u_js = self.get_ik_solution("uf_arm", u_goal)
        if u_js:
            self.execute_joint_trajectory("uf_arm", self.create_constraints_from_joints(u_js, "u1"))
            self.motion_done_event.wait(timeout=25.0)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.motion_done_event.set()
            return
        handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.last_result = future.result().result.error_code.val
        if self.last_result == 1: self.get_logger().info("✅ Success")
        else: self.get_logger().error(f"❌ Failed: {self.last_result}")
        self.motion_done_event.set()

def main(args=None):
    rclpy.init(args=args)
    commander = UniversalCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(commander)

    # TARGETS
    x_target = {'x': 0.85, 'y': 0.04, 'z': 1.07, 'link': "xarm5_tool0"}
    u_target = {
        'x': 0.82887, 'y': -0.20629, 'z': 1.0172, 
        'qx': 0.55531, 'qy': 0.52615, 'qz': 0.46753, 'qw': -0.44295,
        'link': "u1_tool0"
    }

    thread = threading.Thread(target=commander.move_dual_arms_sequentially, arACCgs=(x_target, u_target))
    thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()