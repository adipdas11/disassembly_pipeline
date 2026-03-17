#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs 
import threading, math, time, os, subprocess, json


class MotionBackend:
    def __init__(self, node: Node, group_name: str):
        self.node = node
        self.group_name = group_name

        group_name_lower = group_name.lower()
        self.is_xarm5 = "xarm" in group_name_lower
        self.is_gripper = "rg6" in group_name_lower or "gripper" in group_name_lower
        self.is_uf850 = not self.is_xarm5 and not self.is_gripper

        if self.is_xarm5:
            self.backend_kind = "xarm5"
            self.prefix = "xarm"
            self.controller_name = "xarm_controller"
            self.servo_namespace = "/xarm_servo_node"
            self.default_ik_link = "xarm5_link5"
            self.joint_prefixes = ["xarm5", "slider"]
        elif self.is_gripper:
            self.backend_kind = "rg6_gripper"
            self.prefix = "rg6"
            self.controller_name = "rg6_controller"
            self.servo_namespace = None
            self.default_ik_link = None
            self.joint_prefixes = ["rg6"]
        else:
            self.backend_kind = "uf850"
            self.prefix = "uf"
            self.controller_name = "uf_controller"
            self.servo_namespace = "/uf_servo_node"
            self.default_ik_link = "u1_tool0"
            self.joint_prefixes = ["u1"]

        self.node.get_logger().info(
            f"✅ MotionBackend: Initialized for {self.backend_kind} group '{self.group_name}'."
        )
        
        # --- Reentrant callback group for service clients ---
        # Prevents deadlocks in multi-node executor setups (master_agent)
        # by allowing service responses to be processed in parallel with other callbacks.
        self._service_cb_group = ReentrantCallbackGroup()

        # --- ROS 2 Interfaces ---
        self._action_client = ActionClient(self.node, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self.node, ExecuteTrajectory, 'execute_trajectory')
        self._ik_client = self.node.create_client(GetPositionIK, 'compute_ik', callback_group=self._service_cb_group)
        self._cartesian_client = self.node.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self._service_cb_group)
        self._gripper_force_pub = None
        self.current_gripper_state = {}
        self.current_gripper_width_mm = 0.0
        self.current_gripper_force_n = 0.0
        self.current_gripper_status_raw = 0
        self.default_gripper_force_n = 40.0
        if self.is_gripper:
            self._gripper_force_pub = self.node.create_publisher(Float32, '/rg6/force_command', 10)
            self.node.create_subscription(String, '/rg6/state', self._gripper_state_callback, 10)
        
        # --- MoveIt Servo Publisher ---
        self.servo_pub = None
        self._servo_start_client = None
        self._servo_stop_client = None
        if self.servo_namespace is not None:
            self.servo_pub = self.node.create_publisher(
                TwistStamped, f'{self.servo_namespace}/delta_twist_cmds', 10
            )
            self._servo_start_client = self.node.create_client(
                Trigger, f'{self.servo_namespace}/start_servo',
                callback_group=self._service_cb_group
            )
            self._servo_stop_client = self.node.create_client(
                Trigger, f'{self.servo_namespace}/stop_servo',
                callback_group=self._service_cb_group
            )
        
        # --- TF2 Transformation Engine ---
        # Share one TF buffer/listener per node to avoid:
        #   1. Multiple executor conflicts (spin_thread=True crashes)
        #   2. N*2 duplicate /tf subscriptions starving the executor
        if not hasattr(self.node, '_shared_tf_buffer'):
            self.node._shared_tf_buffer = Buffer()
            self.node._shared_tf_listener = TransformListener(self.node._shared_tf_buffer, self.node)
        self.tf_buffer = self.node._shared_tf_buffer

        # --- Internal State Tracking ---
        self.current_joint_positions = {}
        self.current_joint_efforts = {}
        self.state_received = threading.Event()
        self.joint_sub = self.node.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10,
            callback_group=self._service_cb_group
        )
        
        self.is_activated = False
        self._in_servo_mode = False

    def _gripper_state_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.node.get_logger().warning(f"Failed to parse /rg6/state payload: {exc}")
            return
        self.current_gripper_state = data
        self.current_gripper_width_mm = float(data.get("width_mm", 0.0))
        self.current_gripper_force_n = float(data.get("target_force_n", self.current_gripper_force_n))
        self.current_gripper_status_raw = int(data.get("status_raw", 0))

    def set_gripper_force(self, force_n: float) -> bool:
        if not self.is_gripper or self._gripper_force_pub is None:
            self.node.get_logger().error(f"❌ Gripper force control is unsupported for {self.group_name}.")
            return False
        force_msg = Float32()
        force_msg.data = float(force_n)
        self.default_gripper_force_n = float(force_n)
        self.current_gripper_force_n = float(force_n)
        self._gripper_force_pub.publish(force_msg)
        return True

    def move_gripper(self, joint_position_rad: float, force_n: float = None, velocity: float = 0.2) -> bool:
        if not self.is_gripper:
            self.node.get_logger().error(f"❌ move_gripper() is only valid for gripper groups, not {self.group_name}.")
            return False
        requested_force = self.default_gripper_force_n if force_n is None else float(force_n)
        return self.move_to_joint_positions(
            {"rg6_l_out": float(joint_position_rad)},
            velocity=velocity,
            gripper_force_n=requested_force,
        )

    def _servo_supported(self) -> bool:
        return self.servo_pub is not None and self._servo_start_client is not None and self._servo_stop_client is not None

    def _publish_zero_twist(self):
        if self.servo_pub is None:
            return
        msg = TwistStamped()
        msg.header.frame_id = "world_world"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.servo_pub.publish(msg)

    def _call_trigger_sync(self, client, timeout_sec: float, label: str, max_retries: int = 3):
        if client is None:
            self.node.get_logger().error(f"❌ {label}: servo is not configured for {self.group_name}.")
            return False
        for attempt in range(1, max_retries + 1):
            if not client.wait_for_service(timeout_sec=max(timeout_sec, 5.0)):
                self.node.get_logger().warn(f"⚠️ {label}: service unavailable (attempt {attempt}/{max_retries}).")
                continue
            future = client.call_async(Trigger.Request())
            call_timeout = max(timeout_sec, 5.0)
            deadline = time.time() + call_timeout
            while rclpy.ok() and not future.done():
                if time.time() >= deadline:
                    break
                time.sleep(0.01)
            if not future.done():
                self.node.get_logger().warn(f"⚠️ {label}: service call timeout (attempt {attempt}/{max_retries}).")
                continue
            response = future.result()
            if response is None:
                self.node.get_logger().warn(f"⚠️ {label}: empty response (attempt {attempt}/{max_retries}).")
                continue
            if not response.success:
                self.node.get_logger().warn(f"⚠️ {label}: {response.message} (attempt {attempt}/{max_retries}).")
                continue
            return True
        self.node.get_logger().error(f"❌ {label}: failed after {max_retries} attempts.")
        return False

    def _configure_move_group_request(self, goal, velocity):
        """Apply consistent planning bounds so failed plans return promptly."""
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = velocity
        goal.request.max_acceleration_scaling_factor = velocity

    def _joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]
            if len(msg.effort) > i: 
                self.current_joint_efforts[name] = msg.effort[i]
        self.state_received.set()

    # --- Hardware Management ---
    def reset_robot(self):
        """Asynchronously clears hardware errors and activates controllers."""
        self.node.get_logger().info(f"🔄 Resetting {self.group_name}...")

        def call_service_cmd(cmd):
            try:
                subprocess.run(cmd, shell=True, timeout=3.0, capture_output=True)
                return True
            except Exception:
                return False

        # Clear Errors, Set Mode 0 (Position), Set State 0 (Ready)
        call_service_cmd(f"ros2 service call /{self.prefix}/clear_err std_srvs/srv/Empty {{}}")
        call_service_cmd(f"ros2 service call /{self.prefix}/set_mode xarm_msgs/srv/SetInt16 \"{{data: 0}}\"")
        call_service_cmd(f"ros2 service call /{self.prefix}/set_state xarm_msgs/srv/SetInt16 \"{{data: 0}}\"")
        
        # Re-activate the primary controller
        call_service_cmd(f"ros2 control set_controller_state {self.controller_name} active")
        
        self.is_activated = True
        self.node.get_logger().info(f"✅ Reset Sequence Dispatched for {self.group_name}.")

    def start_servo(self, timeout_sec=5.0):
        """Public: start servo mode with proper state tracking."""
        return self._ensure_servo_mode()

    def stop_servo(self, timeout_sec=5.0):
        """Public: stop servo mode with proper state tracking."""
        return self._ensure_trajectory_mode()

    def stop_immediately(self):
        """Immediately stops all Servo and MoveIt motion.
        Marks servo as inactive since MoveIt Servo will internally timeout
        after receiving zero-twist, making the flag stale."""
        self._publish_zero_twist()
        self._in_servo_mode = False
        self.node.get_logger().error("🛑 MOTION STOPPED")

    # --- Controller Mode Switching ---
    # IMPORTANT: MoveIt Servo publishes TO the JointTrajectoryController.
    # We must NEVER deactivate the trajectory controller — if we do, servo
    # commands have no subscriber and the robot won't move.
    # Instead, we prevent fights by:
    #   1. Blocking until planned trajectories complete (_execute_joint_goal)
    #   2. Flushing servo with zero-twist before planned motions
    #   3. Tracking mode state to avoid redundant flushes

    def _ensure_servo_mode(self):
        """Prepare for servo jogging. The trajectory controller stays active
        (servo publishes through it). We just track the mode."""
        if not self._servo_supported():
            self.node.get_logger().error(f"❌ Servo mode unsupported for {self.group_name}.")
            return False
        if self._in_servo_mode:
            return True
        # Clean slate: stop servo first (best-effort, ignores failure).
        # This handles the case where stop_immediately() only published
        # zero-twist without calling the stop service, leaving servo in
        # an ambiguous internal state where start_servo would fail with
        # "already running".
        if self._servo_stop_client is not None:
            try:
                if self._servo_stop_client.wait_for_service(timeout_sec=1.0):
                    future = self._servo_stop_client.call_async(Trigger.Request())
                    deadline = time.time() + 2.0
                    while rclpy.ok() and not future.done():
                        if time.time() >= deadline:
                            break
                        time.sleep(0.01)
            except Exception:
                pass
            time.sleep(0.1)
        if not self._call_trigger_sync(self._servo_start_client, timeout_sec=2.0, label="start_servo"):
            return False
        self._in_servo_mode = True
        # Warm-up: MoveIt Servo needs time after start for its internal twist
        # subscriber to match the existing publisher (DDS discovery latency).
        # Publish zero-twist for 0.5s to ensure servo is processing commands
        # before any caller publishes real twist velocities.
        for _ in range(15):
            self._publish_zero_twist()
            time.sleep(0.033)
        self.node.get_logger().info(f"🔄 Entering servo mode (warmed up).")
        return True

    def _ensure_trajectory_mode(self):
        """Prepare for planned trajectory execution. Flush any lingering servo
        commands by sending zero-twist, then explicitly stop the servo node
        to switch the xArm driver back to position control mode.

        Always stops servo via service call (best-effort) even if _in_servo_mode
        is False, because stop_immediately() and retract_servo_z_closed_loop
        clear the flag without calling the stop service — leaving servo
        internally running and interfering with planned trajectory execution.
        """
        if not self._servo_supported():
            self._in_servo_mode = False
            return True

        # Always flush and stop — servo may be running even if flag is False
        self._publish_zero_twist()
        time.sleep(0.1)
        if self._servo_stop_client is not None:
            try:
                if self._servo_stop_client.wait_for_service(timeout_sec=1.0):
                    future = self._servo_stop_client.call_async(Trigger.Request())
                    deadline = time.time() + 2.0
                    while rclpy.ok() and not future.done():
                        if time.time() >= deadline:
                            break
                        time.sleep(0.01)
            except Exception:
                pass
        self._in_servo_mode = False
        self.node.get_logger().info(f"🔄 Entering trajectory mode (servo stopped).")
        return True

    # --- Core Motion Logic ---
    def move_to_pose_robust(self, x, y, z, q_dict=None, velocity=0.1, frame_id='world_world'):
        if not self._ensure_trajectory_mode():
            return False
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.avoid_collisions = True
        
        # 🦾 THE FIX: Strictly align the tip frame with what MoveIt expects
        # Your log says only [xarm5_link5] is available for this group.
        req.ik_request.ik_link_name = self.default_ik_link
        
        ps = PoseStamped()
        ps.header.frame_id, ps.header.stamp = frame_id, self.node.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        
        # 5-DOF arms usually need Roll=PI (pointed down)
        if q_dict:
            ps.pose.orientation = Quaternion(x=q_dict['qx'], y=q_dict['qy'], z=q_dict['qz'], w=q_dict['qw'])
        else:
            ps.pose.orientation = self._rpy_to_quaternion(math.pi, 0.0, 0.0)

        self.node.get_logger().info(
            f"🎯 Pose request for {self.group_name}: "
            f"pos=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"quat=({ps.pose.orientation.x:.3f}, {ps.pose.orientation.y:.3f}, "
            f"{ps.pose.orientation.z:.3f}, {ps.pose.orientation.w:.3f}) in {frame_id}"
        )

        req.ik_request.pose_stamped = ps
        req.ik_request.robot_state = self._get_full_robot_state()
        
        # Primary Attempt
        ik_res = self._call_ik_sync(req)
        if ik_res.error_code.val == 1:
            if not self._execute_joint_goal(ik_res.solution.joint_state, velocity):
                return False
            return True
        self.node.get_logger().warn(
            f"⚠️ IK failed for {self.group_name} with error code {ik_res.error_code.val}."
        )

        # 5-DOF Yaw Sweep Fallback (xarm5_link5)
        if self.is_xarm5:
            self.node.get_logger().warn("⚠️ IK Failed for xarm5_link5. Attempting Yaw Sweep...")
            for yaw_deg in [15, -15, 30, -30, 45, -45, 90, -90, 180]:
                yaw_rad = math.radians(yaw_deg)
                ps.pose.orientation = self._rpy_to_quaternion(math.pi, 0.0, yaw_rad)
                req.ik_request.pose_stamped = ps
                req.ik_request.robot_state = self._get_full_robot_state()
                ik_res = self._call_ik_sync(req)
                if ik_res.error_code.val == 1:
                    self.node.get_logger().info(f"✅ IK Found at Yaw: {yaw_deg}°")
                    if not self._execute_joint_goal(ik_res.solution.joint_state, velocity):
                        return False
                    return True
        
        return False

    def move_cartesian_to_pose(self, x, y, z, q_dict=None, velocity=0.1, frame_id='world_world'):
        """Move to target using Cartesian path planning (straight-line in task space).
        Solves IK incrementally along the path — works where single-shot IK fails on 5-DOF arms."""
        if not self._ensure_trajectory_mode():
            return False
        target_link = self.default_ik_link
        current_tf = None

        target = Pose()
        target.position.x, target.position.y, target.position.z = x, y, z

        if q_dict:
            target.orientation = Quaternion(x=q_dict['qx'], y=q_dict['qy'], z=q_dict['qz'], w=q_dict['qw'])
        else:
            # Keep current EE orientation — safest for 5-DOF arms
            try:
                current_tf = self.tf_buffer.lookup_transform(frame_id, target_link, rclpy.time.Time())
                q = current_tf.transform.rotation
                target.orientation = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
            except Exception:
                target.orientation = self._rpy_to_quaternion(math.pi, 0.0, 0.0)

        if current_tf is None:
            try:
                current_tf = self.tf_buffer.lookup_transform(frame_id, target_link, rclpy.time.Time())
            except Exception as e:
                self.node.get_logger().warn(f"⚠️ Failed to read current pose before Cartesian plan: {e}")

        linear_distance = None
        if current_tf is not None:
            dx = x - current_tf.transform.translation.x
            dy = y - current_tf.transform.translation.y
            dz = z - current_tf.transform.translation.z
            linear_distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        req = GetCartesianPath.Request()
        req.header.frame_id = frame_id
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.start_state = self._get_full_robot_state()
        req.group_name = self.group_name
        req.link_name = target_link
        req.waypoints = [target]
        req.max_step = 0.005  # 5mm interpolation resolution
        req.jump_threshold = 0.0  # Disable jump detection (unreliable for 5-DOF)
        req.avoid_collisions = True

        dist_msg = f", distance={linear_distance:.3f}m" if linear_distance is not None else ""
        self.node.get_logger().info(
            f"🦾 Planning Cartesian path to ({x:.3f}, {y:.3f}, {z:.3f}){dist_msg}..."
        )

        if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error("❌ compute_cartesian_path service unavailable.")
            return False

        future = self._cartesian_client.call_async(req)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > 15.0:
                self.node.get_logger().error("❌ Cartesian path service timeout (15s).")
                return False
            time.sleep(0.01)

        result = future.result()
        if result.fraction < 0.90:
            self.node.get_logger().warn(f"⚠️ Cartesian path only {result.fraction*100:.0f}% feasible.")
            return False

        joint_points = result.solution.joint_trajectory.points
        mdof_points = result.solution.multi_dof_joint_trajectory.points
        self.node.get_logger().info(
            f"🧭 Cartesian trajectory contains {len(joint_points)} joint points and "
            f"{len(mdof_points)} multi-DOF points."
        )

        if not joint_points and not mdof_points:
            self.node.get_logger().warn("⚠️ Cartesian planner returned an empty trajectory.")
            return False

        if len(joint_points) <= 1 and linear_distance is not None and linear_distance > 0.01:
            self.node.get_logger().warn(
                "⚠️ Cartesian planner returned a trivial trajectory for a non-trivial move."
            )
            return False

        self.node.get_logger().info(f"✅ Cartesian path {result.fraction*100:.0f}% feasible. Executing...")

        # Execute the planned trajectory
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.solution

        if not self._execute_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error("❌ ExecuteTrajectory action server unavailable.")
            return False

        future = self._execute_client.send_goal_async(exec_goal)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > 30.0:
                self.node.get_logger().error("❌ Cartesian trajectory acceptance timeout (30s).")
                return False
            time.sleep(0.01)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("❌ Cartesian trajectory rejected.")
            return False

        result_future = goal_handle.get_result_async()
        t0 = time.time()
        while not result_future.done():
            if time.time() - t0 > 60.0:
                self.node.get_logger().error("❌ Cartesian execution timeout (60s).")
                return False
            time.sleep(0.01)

        exec_result = result_future.result().result
        success = exec_result.error_code.val == 1
        if success:
            self.node.get_logger().info("✅ Cartesian move complete.")
        else:
            self.node.get_logger().warn(f"⚠️ Cartesian execution error (code: {exec_result.error_code.val}).")
        return success

    def move_to_joint_positions(self, target_joints, velocity=0.2, gripper_force_n=None):
        if not self._ensure_trajectory_mode():
            return False
        if not self.state_received.wait(timeout=2.0):
            self.node.get_logger().error("❌ Timed out waiting for /joint_states before joint move.")
            return False
        if self.is_gripper and gripper_force_n is not None and not self.set_gripper_force(gripper_force_n):
            return False

        goal = MoveGroup.Goal()
        self._configure_move_group_request(goal, velocity)
        
        constraints = Constraints()
        prefixes = self.joint_prefixes
        
        found_joints = False
        for name, pos in target_joints.items():
            # Check if current joint in target_joints matches the arm we are controlling
            if any(name.startswith(pfx) for pfx in prefixes):
                jc = JointConstraint()
                jc.joint_name, jc.position, jc.weight = name, float(pos), 1.0
                jc.tolerance_above = jc.tolerance_below = 0.01
                constraints.joint_constraints.append(jc)
                found_joints = True
        
        if not found_joints:
            self.node.get_logger().error(f"❌ Prefix mismatch: {prefixes} not found in target_joints.")
            return False

        goal.request.goal_constraints.append(constraints)
        
        self.node.get_logger().info(f"🦾 Sending Joint Goal for {self.group_name}...")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error("❌ MoveGroup action server unavailable.")
            return False
        
        # 2. Send Goal and wait for result
        future = self._action_client.send_goal_async(goal)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > 30.0:
                self.node.get_logger().error("❌ Joint Goal acceptance timeout (30s).")
                return False
            time.sleep(0.01)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("❌ Joint Goal Rejected by MoveIt.")
            return False

        result_future = goal_handle.get_result_async()
        t0 = time.time()
        while not result_future.done():
            if time.time() - t0 > 60.0:
                self.node.get_logger().error("❌ Joint Goal execution timeout (60s).")
                return False
            time.sleep(0.01)
            
        success = result_future.result().result.error_code.val == 1
        if success:
            self.node.get_logger().info("✅ Joint Move Complete.")
        else:
            self.node.get_logger().warn(
                f"⚠️ Joint Move failed with code {result_future.result().result.error_code.val}."
            )
        return success

    def move_linear_z_with_torque_stop(self, speed_mps, threshold_nm, joint_index=2, timeout=3600.0):
        """Tactile descent using MoveIt Servo and baseline-subtraction monitoring."""
        if not self._ensure_servo_mode():
            return False
        arm_pfx = 'xarm5' if self.is_xarm5 else 'u1'
        joint_name = f"{arm_pfx}_joint{joint_index+1}"
        target_link = self.default_ik_link

        if joint_name not in self.current_joint_efforts:
            self.node.get_logger().error(f"❌ No effort feedback available for {joint_name}.")
            return False

        try:
            start_z = self.tf_buffer.lookup_transform("world_world", target_link, rclpy.time.Time()).transform.translation.z
        except Exception as exc:
            self.node.get_logger().error(f"❌ Unable to read start pose for tactile descent: {exc}")
            return False

        time.sleep(0.1)
        effort_samples = []
        sample_deadline = time.time() + 0.15
        while time.time() < sample_deadline:
            effort_samples.append(self.current_joint_efforts.get(joint_name, 0.0))
            time.sleep(0.01)
        baseline = sum(effort_samples) / len(effort_samples) if effort_samples else self.current_joint_efforts.get(joint_name, 0.0)
        
        twist = TwistStamped()
        twist.header.frame_id = "world_world"
        twist.twist.linear.z = -abs(speed_mps)
        
        start_t = time.time()

        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = self.current_joint_efforts.get(joint_name, 0.0)
            spike = abs(curr - baseline)
            
            # Debug: Throttle print the live spike
            if int((time.time() - start_t) * 10) % 5 == 0:
                self.node.get_logger().info(f"📊 Live Spike on {joint_name}: {spike:.3f} Nm", throttle_duration_sec=0.3)

            # Blanking period (0.2s) to ignore initial jerk
            if (time.time() - start_t) > 0.2:
                # Reduced multiplier to 2.0x (3.0 * 2.0 = 6Nm). 15Nm was too high to trigger reliably.
                if spike > (threshold_nm * 2.0):
                    # Stop motion but keep servo mode alive so the next
                    # servo operation (e.g. retract) can continue without
                    # a full stop/restart cycle that can fail on back-to-back calls.
                    self._publish_zero_twist()
                    self.node.get_logger().warn(f"🎯 CONTACT DETECTED: {spike:.3f}Nm spike (Threshold: {threshold_nm*2.0}Nm).")
                    return True

            # Removed distance-based abort (max_travel_m) as requested.
            # Motion will now continue until contact or timeout.

            twist.header.stamp = self.node.get_clock().now().to_msg()
            self.servo_pub.publish(twist)
            time.sleep(0.033)
        self.stop_immediately()
        self.node.get_logger().error("❌ Tactile descent timed out before contact.")
        return False
    
    def retract_relative_z(self, distance, velocity=0.05):
        """Planned relative lift along the Z-axis."""
        target_link = "xarm5_link5" if self.is_xarm5 else "u1_tool0"
        try:
            current_tf = self.tf_buffer.lookup_transform('world_world', target_link, rclpy.time.Time())
            tx = current_tf.transform.translation.x
            ty = current_tf.transform.translation.y
            tz = current_tf.transform.translation.z + distance
            q = current_tf.transform.rotation
            q_dict = {'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
            return self.move_to_pose_robust(tx, ty, tz, q_dict, velocity=velocity)
        except Exception as e:
            self.node.get_logger().error(f"Retract TF Error: {e}"); return False

    def jog_cartesian_servo(self, dx, dy, dz, duration=1.0):
        """Fine-grained cartesian jogging via MoveIt Servo."""
        if not self._ensure_servo_mode():
            return False
        twist = TwistStamped(); twist.header.frame_id = "world_world"
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = dx, dy, dz
        end_t = time.time() + duration
        while rclpy.ok() and time.time() < end_t:
            twist.header.stamp = self.node.get_clock().now().to_msg()
            self.servo_pub.publish(twist)
            time.sleep(0.033)  # Match servo publish_period (0.03s / ~30Hz)
        self._publish_zero_twist()
        return True
            
    def get_transformed_pose(self, source_pose, source_frame, target_frame, z_offset=0.0):
        try:
            p = PoseStamped()
            p.header.frame_id = source_frame
            # 🦾 FIX: Use Time() (zero) to get the latest available transform
            p.header.stamp = rclpy.time.Time().to_msg() 
            
            p.pose = source_pose.pose if hasattr(source_pose, 'pose') else source_pose
            
            # Check if transform is possible before attempting
            if not self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), 
                                              timeout=rclpy.duration.Duration(seconds=1.0)):
                return None
                
            t = self.tf_buffer.transform(p, target_frame)
            t.pose.position.z += z_offset
            return t
        except Exception as e:
            self.node.get_logger().error(f"TF Error: {e}")
            return None

    def retract_servo_z_closed_loop(self, distance, speed_mps=0.03, timeout=None):
        """Closed-loop Z retract using TF feedback.

        Publishes twist and monitors TF continuously. Stops when TF confirms
        the target distance (with 2mm tolerance for TF lag). Uses stall
        detection to avoid waiting forever if the robot physically can't
        move further. Returns True if >= 85% of distance was achieved.
        """
        if not self._ensure_servo_mode():
            return False

        target_link = self.default_ik_link
        abs_distance = abs(distance)
        # No hard cap — closed-loop TF feedback with tolerance and stall
        # detection prevents overshoot. MoveIt Servo internally scales
        # velocity near singularities/limits anyway.
        cmd_speed = speed_mps
        # MoveIt Servo often delivers only ~2-5mm/s effective speed due to
        # internal velocity scaling near singularities/limits.
        # Generous timeout: assume worst-case 1mm/s + 5s buffer.
        if timeout is None:
            timeout = max(abs_distance / 0.001, 10.0) + 5.0

        try:
            start_tf = self.tf_buffer.lookup_transform('world_world', target_link, rclpy.time.Time())
            start_z = start_tf.transform.translation.z
        except Exception as e:
            self.node.get_logger().error(f"Z Retract TF Init Error: {e}")
            return False

        self.node.get_logger().info(
            f"🔄 Servo Z retract: {abs_distance*1000:.1f}mm at {cmd_speed*1000:.1f}mm/s "
            f"(start_z={start_z:.4f}, timeout={timeout:.0f}s)")

        twist = TwistStamped()
        twist.header.frame_id = "world_world"
        twist.twist.linear.z = cmd_speed if distance > 0 else -cmd_speed

        # TF tolerance: accept within 2mm of target (TF lags ~2-3mm behind)
        tf_tolerance = 0.002
        # Stall detection: if TF shows no movement for 3s, robot is stuck
        last_z = start_z
        last_change_t = time.time()
        stall_threshold = 0.0005  # 0.5mm minimum change to count as moving

        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t) < timeout:
            try:
                curr_tf = self.tf_buffer.lookup_transform('world_world', target_link, rclpy.time.Time())
                curr_z = curr_tf.transform.translation.z
                traveled = abs(curr_z - start_z)

                # Target reached (with tolerance for TF lag)
                if traveled >= abs_distance - tf_tolerance:
                    self._publish_zero_twist()
                    self._in_servo_mode = False
                    self.node.get_logger().info(
                        f"✅ Z retract complete at z={curr_z:.4f} "
                        f"(moved {traveled*1000:.1f}mm)")
                    return True

                # Stall detection: if position hasn't changed in 3s, stop early
                if abs(curr_z - last_z) > stall_threshold:
                    last_z = curr_z
                    last_change_t = time.time()
                elif (time.time() - last_change_t) > 3.0 and traveled > 0.001:
                    self._publish_zero_twist()
                    self._in_servo_mode = False
                    pct = (traveled / abs_distance) * 100
                    self.node.get_logger().info(
                        f"✅ Z retract stall-stop at z={curr_z:.4f} "
                        f"(moved {traveled*1000:.1f}mm, {pct:.0f}% of target)")
                    return traveled >= abs_distance * 0.85

            except Exception:
                pass

            twist.header.stamp = self.node.get_clock().now().to_msg()
            self.servo_pub.publish(twist)
            time.sleep(0.033)

        self._publish_zero_twist()
        self._in_servo_mode = False
        # On timeout: check if close enough (>= 85% is acceptable)
        try:
            end_tf = self.tf_buffer.lookup_transform('world_world', target_link, rclpy.time.Time())
            end_z = end_tf.transform.translation.z
            actual_mm = abs(end_z - start_z) * 1000
            pct = (actual_mm / (abs_distance * 1000)) * 100
            close_enough = actual_mm >= abs_distance * 1000 * 0.85
            level = "info" if close_enough else "warn"
            msg = (f"Z retract {'complete' if close_enough else 'timeout'} "
                   f"(moved {actual_mm:.1f}mm of {abs_distance*1000:.1f}mm, {pct:.0f}%)")
            if close_enough:
                self.node.get_logger().info(f"✅ {msg}")
            else:
                self.node.get_logger().warn(f"⚠️ {msg}")
            return close_enough
        except Exception:
            self.node.get_logger().warn(f"⚠️ Z retract timeout ({timeout:.0f}s)")
            return False
    
    def move_servo_xy_closed_loop(self, dx, dy, speed_mps=0.03, timeout=30.0, stop_check=None):
        """Moves a specific relative distance in World XY using TF feedback with optional stop condition."""
        if not self._ensure_servo_mode():
            return False
        
        target_link = self.default_ik_link
        try:
            start_tf = self.tf_buffer.lookup_transform('world_world', target_link, rclpy.time.Time())
            sx, sy = start_tf.transform.translation.x, start_tf.transform.translation.y
            target_dist = math.hypot(dx, dy)
        except Exception as e:
            self.node.get_logger().error(f"XY Servo Init Error: {e}")
            return False

        self.node.get_logger().info(f"🔄 Servo XY: Moving {target_dist*1000:.1f}mm in World XY at {speed_mps*1000:.1f}mm/s")

        twist = TwistStamped()
        twist.header.frame_id = "world_world"
        
        if target_dist > 0:
            twist.twist.linear.x = (dx / target_dist) * speed_mps
            twist.twist.linear.y = (dy / target_dist) * speed_mps
        else:
            return True

        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t) < timeout:
            # Check external stop condition (e.g. vision regained)
            if stop_check and stop_check():
                self._publish_zero_twist()
                self.node.get_logger().info("🛑 XY Servo stopped by external condition.")
                return "STOPPED"

            try:
                curr_tf = self.tf_buffer.lookup_transform('world_world', target_link, rclpy.time.Time())
                cx, cy = curr_tf.transform.translation.x, curr_tf.transform.translation.y
                
                traveled = math.hypot(cx - sx, cy - sy)
                if traveled >= target_dist:
                    self._publish_zero_twist()
                    self.node.get_logger().info(f"✅ XY Servo complete (Moved {traveled*1000:.1f}mm)")
                    return True
            except Exception:
                pass
            
            twist.header.stamp = self.node.get_clock().now().to_msg()
            self.servo_pub.publish(twist)
            time.sleep(0.033)
            
        self._publish_zero_twist()
        self.node.get_logger().warn(f"⚠️ XY Servo timeout ({timeout}s)")
        return False
    
    def _execute_joint_goal(self, js, vel):
        """Standardizes joint execution across arms, grippers, and sliders."""
        goal = MoveGroup.Goal()
        self._configure_move_group_request(goal, vel)

        constraints = Constraints()

        # 🎯 THE FIX: Isolate the Gripper from the Arm
        prefixes = self.joint_prefixes

        found_any = False
        for n, p in zip(js.name, js.position):
            # Check if the joint name starts with any of our valid prefixes
            if any(n.startswith(pfx) for pfx in prefixes):
                jc = JointConstraint()
                jc.joint_name, jc.position, jc.weight = n, p, 1.0
                jc.tolerance_above = jc.tolerance_below = 0.01 # Added tolerance for safety
                constraints.joint_constraints.append(jc)
                found_any = True

        if not found_any:
            self.node.get_logger().error(f"❌ No joints matching {prefixes} found in message!")
            return False

        goal.request.goal_constraints.append(constraints)

        self.node.get_logger().info(f"🦾 Sending Pose Goal for {self.group_name}...")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error("❌ MoveGroup action server unavailable.")
            return False

        # Wait for goal acceptance (timeout: 30s)
        future = self._action_client.send_goal_async(goal)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > 30.0:
                self.node.get_logger().error("❌ Pose Goal acceptance timeout (30s).")
                return False
            time.sleep(0.01)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("❌ Pose Goal Rejected by MoveIt.")
            return False

        # Wait for execution (timeout: 60s)
        result_future = goal_handle.get_result_async()
        t0 = time.time()
        while not result_future.done():
            if time.time() - t0 > 60.0:
                self.node.get_logger().error("❌ Pose Goal execution timeout (60s).")
                return False
            time.sleep(0.01)

        success = result_future.result().result.error_code.val == 1
        if success:
            self.node.get_logger().info("✅ Pose Goal Complete.")
        else:
            self.node.get_logger().warn(
                f"⚠️ Pose Goal execution failed with code "
                f"{result_future.result().result.error_code.val}."
            )
        return success

    def _call_ik_sync(self, req):
        future = self._ik_client.call_async(req)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > 10.0:
                self.node.get_logger().error("❌ IK service timeout (10s).")
                return type('FakeResult', (), {'error_code': type('EC', (), {'val': -1})()})()
            time.sleep(0.01)
        return future.result()

    def _get_full_robot_state(self):
        state = RobotState(); js = JointState(); js.header.stamp = self.node.get_clock().now().to_msg()
        js.name, js.position = list(self.current_joint_positions.keys()), list(self.current_joint_positions.values())
        state.joint_state = js; return state

    def _rpy_to_quaternion(self, r, p, y):
        cy, sy = math.cos(y*0.5), math.sin(y*0.5)
        cp, sp = math.cos(p*0.5), math.sin(p*0.5)
        cr, sr = math.cos(r*0.5), math.sin(r*0.5)
        return Quaternion(w=cr*cp*cy+sr*sp*sy, x=sr*cp*cy-cr*sp*sy, y=cr*sp*cy+sr*cp*sy, z=cr*cp*sy-sr*sp*cy)
