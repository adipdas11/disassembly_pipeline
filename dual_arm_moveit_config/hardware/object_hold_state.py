#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

class ObjectHoldStateManager(Node):
    """
    Central State Manager for the Gripper's Hold Status.

    Listens to the hold skill, caches the current state, and broadcasts 
    updates to the rest of the ROS 2 network.
    """
    def __init__(self):
        super().__init__('object_hold_state_manager')
        
        # --- Internal Memory ---
        self.is_holding = False
        self.state_string = "EMPTY"
        
        # --- Subscriptions ---
        # Listens to the hold state published by skills (TRANSIENT_LOCAL to get latched values)
        self.hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool,
            '/object_hold_state/is_held',
            self.hold_status_callback,
            self.hold_qos
        )

        # --- Publishers ---
        # Publishes a human-readable string for UI or state machines (e.g., "HOLDING", "EMPTY")
        self.string_pub = self.create_publisher(String, '/object_hold_state/update', 10)

        self.get_logger().info("✅ Object Hold State Manager Initialized. Current State: EMPTY")

    def hold_status_callback(self, msg: Bool):
        """
        Handle object hold state changes from the grasp pipeline.
        """
        # Only log and update if the state actually changes
        if self.is_holding != msg.data:
            self.is_holding = msg.data
            self.state_string = "HOLDING" if self.is_holding else "EMPTY"
            
            if self.is_holding:
                self.get_logger().info("📦 State Changed: Now HOLDING an object.")
            else:
                self.get_logger().info("👐 State Changed: Gripper is now EMPTY.")
                
            # Force an immediate broadcast on change
            self.broadcast_state()

    def broadcast_state(self):
        """Publish the cached state to the String topic."""
        str_msg = String()
        str_msg.data = self.state_string
        self.string_pub.publish(str_msg)

def main(args=None):
    rclpy.init(args=args)
    state_node = ObjectHoldStateManager()
    
    try:
        rclpy.spin(state_node)
    except KeyboardInterrupt:
        state_node.get_logger().info("Shutting down Hold State Manager...")
    finally:
        try:
            state_node.destroy_node()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()
