#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DisassemblyStateManager(Node):
    def __init__(self):
        super().__init__('disassembly_state_manager')

        # State Definitions
        self.VALID_STATES = [
            "IDLE", 
            "MOVING", 
            "UNSCREWING", 
            "UNSCREWING_COMPLETE", 
            "UNSCREWING_RETRY", 
            "HOLDING", 
            "FLIPPING",
            "ERROR"
        ]

        # Single synchronized topic for the whole system
        self.state_pub = self.create_publisher(String, '/robot_states', 10)

        # Separate update topics so skills can report their status independently
        self.create_subscription(String, '/robot_state/tool_arm/update', self.tool_update_cb, 10)
        self.create_subscription(String, '/robot_state/manip_arm/update', self.manip_update_cb, 10)

        # Internal State Store
        self.states = {
            "tool_arm": "IDLE",   # xarm5
            "manip_arm": "IDLE"   # uf850
        }

        # Timer to broadcast the combined state at 10Hz
        self.timer = self.create_timer(0.1, self.publish_combined_state)

        self.get_logger().info("✅ Dual-Arm State Manager Online (Topic: /robot_states)")

    def tool_update_cb(self, msg):
        new_state = msg.data.upper()
        if new_state in self.VALID_STATES:
            self.states["tool_arm"] = new_state
        else:
            self.get_logger().error(f"Invalid Tool Arm state: {new_state}")

    def manip_update_cb(self, msg):
        new_state = msg.data.upper()
        if new_state in self.VALID_STATES:
            self.states["manip_arm"] = new_state
        else:
            self.get_logger().error(f"Invalid Manip Arm state: {new_state}")

    def publish_combined_state(self):
        """Publish both states as a JSON string for easy parsing by the agent."""
        msg = String()
        msg.data = json.dumps(self.states)
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DisassemblyStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()
