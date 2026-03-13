#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import ListControllers, SwitchController


class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge_final')

        self.active_arm = 'xarm'
        self.planning_frame = 'world_world'

        # Scales and filters
        self.linear_scale = 1.0
        self.angular_scale = 1.0
        self.joystick_alpha = 0.12
        self.deadzone = 0.08

        # 6-axis state
        self.target_linear = [0.0, 0.0, 0.0]
        self.target_angular = [0.0, 0.0, 0.0]
        self.current_linear = [0.0, 0.0, 0.0]
        self.current_angular = [0.0, 0.0, 0.0]

        # --- Publishers ---
        self.xarm_pub = self.create_publisher(TwistStamped, '/xarm_servo_node/delta_twist_cmds', 10)
        self.uf_pub = self.create_publisher(TwistStamped, '/uf_servo_node/delta_twist_cmds', 10)
        self.gripper_traj_pub = self.create_publisher(JointTrajectory, '/rg6_controller/joint_trajectory', 10)
        self.xarm_traj_pub = self.create_publisher(JointTrajectory, '/xarm_controller/joint_trajectory', 10)
        self.uf_traj_pub = self.create_publisher(JointTrajectory, '/uf_controller/joint_trajectory', 10)
        self.tool_pub = self.create_publisher(Int8, 'tool_cmd', 10)

        # Service clients
        self.cm_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.list_controllers_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.srv_clients = {
            'xarm_start': self.create_client(Trigger, '/xarm_servo_node/start_servo'),
            'uf_start': self.create_client(Trigger, '/uf_servo_node/start_servo'),
        }

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.last_buttons = [0] * 12

        # Internal states
        self.gripper_is_closed = False
        self.unscrew_active = False
        self.grab_active = False
        self.deadman_active = False
        self.pending_home = False

        self.create_timer(0.02, self.continuous_twist_publisher)
        self.sync_timer = self.create_timer(2.0, self.initial_sync_timer_callback)
        self.home_timer = self.create_timer(0.2, self.process_pending_home)
        self.home_timer.cancel()

        self.get_logger().info("Teleop bridge online")

    def initial_sync_timer_callback(self):
        self.manage_servo_nodes()
        self.sync_timer.cancel()

    def manage_servo_nodes(self):
        """Activate inactive controllers and ensure the selected Servo node is started."""
        if not self.cm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("switch_controller service is not available")
            return
        if not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("list_controllers service is not available")
            self.start_active_servo()
            return

        future = self.list_controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(self._handle_list_controllers)

    def _handle_list_controllers(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to list controllers: {exc}")
            self.start_active_servo()
            return

        all_needed = {'xarm_controller', 'uf_controller', 'slider_controller', 'rg6_controller'}
        to_activate = [
            controller.name
            for controller in response.controller
            if controller.name in all_needed and controller.state == 'inactive'
        ]

        if to_activate:
            sw_req = SwitchController.Request()
            sw_req.activate_controllers = to_activate
            sw_req.deactivate_controllers = []
            sw_req.strictness = SwitchController.Request.BEST_EFFORT
            switch_future = self.cm_client.call_async(sw_req)
            switch_future.add_done_callback(self._handle_switch_response)
            self.get_logger().info(f"Activating inactive controllers: {to_activate}")
            return

        self.start_active_servo()

    def _handle_switch_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Controller switch failed: {exc}")
            self.start_active_servo()
            return

        if not response.ok:
            self.get_logger().warning("Controller switch request completed without success")
        self.start_active_servo()

    def start_active_servo(self):
        servo_key = 'xarm_start' if self.active_arm == 'xarm' else 'uf_start'
        servo_label = 'xarm' if self.active_arm == 'xarm' else 'uf850'
        client = self.srv_clients[servo_key]

        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"{servo_label} start_servo service is not available")
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._handle_trigger_response(done, servo_label))

    def _handle_trigger_response(self, future, servo_label):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to start {servo_label} servo: {exc}")
            return

        if response.success:
            self.get_logger().info(f"{servo_label} servo ready")
        else:
            self.get_logger().warning(f"{servo_label} servo rejected start request: {response.message}")

    def joy_callback(self, msg):
        try:
            btns = list(msg.buttons) + [0] * (12 - len(msg.buttons))
            axes = [self.apply_deadzone(axis) for axis in (list(msg.axes) + [0.0] * (8 - len(msg.axes)))]

            # BTN 3 (Y): global homing
            if btns[3] == 1 and self.last_buttons[3] == 0:
                self.get_logger().info("Global home requested")
                self.manage_servo_nodes()
                self.pending_home = True
                self.home_timer.reset()

            # BTN 1 (B): grab / release
            if btns[1] == 1 and self.last_buttons[1] == 0:
                self.grab_active = not self.grab_active
                cmd = 2 if self.grab_active else 3
                self.tool_pub.publish(Int8(data=cmd))
                self.get_logger().info(f"Tool command: {'grab' if self.grab_active else 'release'}")

            # BTN 2 (X): unscrew / stop
            if btns[2] == 1 and self.last_buttons[2] == 0:
                self.unscrew_active = not self.unscrew_active
                cmd = -1 if self.unscrew_active else 0
                self.tool_pub.publish(Int8(data=cmd))
                self.get_logger().info(f"Tool command: {'unscrew' if self.unscrew_active else 'stop'}")

            # BTN 4 (L1): swap robot
            if btns[4] == 1 and self.last_buttons[4] == 0:
                self.active_arm = 'uf' if self.active_arm == 'xarm' else 'xarm'
                self.get_logger().info(f"Active arm switched to {self.active_arm}")
                self.manage_servo_nodes()

            # BTN 5 (R1): gripper open / close
            if btns[5] == 1 and self.last_buttons[5] == 0:
                self.gripper_is_closed = not self.gripper_is_closed
                pos = -0.6 if self.gripper_is_closed else 0.6
                self.get_logger().info(f"Gripper command: {'close' if self.gripper_is_closed else 'open'}")
                self.send_traj_goal(self.gripper_traj_pub, ['rg6_l_out'], [pos], duration=0.6)

            # BTN 0 (A): deadman switch
            if btns[0] == 1:
                self.deadman_active = True
                self.target_linear = [axes[1] * self.linear_scale, axes[0] * self.linear_scale, axes[4] * self.linear_scale]
                self.target_angular = [0.0, 0.0, 0.0]
                self.target_angular[2] = axes[3] * self.angular_scale
                if self.active_arm == 'uf':
                    # D-pad controls for UF850 pitch/roll
                    self.target_angular[1] = axes[7] * self.angular_scale
                    self.target_angular[0] = axes[6] * self.angular_scale
            else:
                self.deadman_active = False
                self.target_linear, self.target_angular = [0.0] * 3, [0.0] * 3

            self.last_buttons = list(btns)
        except Exception as e:
            self.get_logger().error(f"Joy error: {e}")

    def process_pending_home(self):
        if not self.pending_home:
            return

        self.pending_home = False
        self.home_timer.cancel()

        self.send_traj_goal(
            self.xarm_traj_pub,
            ['xarm5_joint1', 'xarm5_joint2', 'xarm5_joint3', 'xarm5_joint4', 'xarm5_joint5'],
            [0.0, 0.0, -1.57, 1.57, 0.0],
        )
        self.send_traj_goal(
            self.uf_traj_pub,
            ['u1_joint1', 'u1_joint2', 'u1_joint3', 'u1_joint4', 'u1_joint5', 'u1_joint6'],
            [0.0, 0.0, -1.57, 0.0, -1.57, 0.0],
        )

    def apply_deadzone(self, value):
        return 0.0 if abs(value) < self.deadzone else value

    def continuous_twist_publisher(self):
        for i in range(3):
            self.current_linear[i] = (self.joystick_alpha * self.target_linear[i]) + ((1.0 - self.joystick_alpha) * self.current_linear[i])
            self.current_angular[i] = (self.joystick_alpha * self.target_angular[i]) + ((1.0 - self.joystick_alpha) * self.current_angular[i])

        if self.deadman_active or any(abs(v) > 0.001 for v in self.current_linear + self.current_angular):
            tw = TwistStamped()
            tw.header.stamp, tw.header.frame_id = self.get_clock().now().to_msg(), self.planning_frame
            tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = self.current_linear
            tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = self.current_angular
            (self.xarm_pub if self.active_arm == 'xarm' else self.uf_pub).publish(tw)

    def send_traj_goal(self, publisher, names, positions, duration=4.0):
        msg = JointTrajectory()
        msg.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        msg.points.append(point)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TeleopBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
