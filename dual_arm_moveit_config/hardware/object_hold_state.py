#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
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
        # Listens to the raw boolean flag from the Object Hold Skill
        self.create_subscription(
            Bool, 
            '/object_hold_status', 
            self.hold_status_callback, 
            10
        )
        
        # --- Publishers ---
        # 1. Publishes the raw boolean for logic/code checks
        self.bool_pub = self.create_publisher(Bool, '/object_hold_state/is_held', 10)
        
        # 2. Publishes a human-readable string for UI or state machines (e.g., "HOLDING", "EMPTY")
        self.string_pub = self.create_publisher(String, '/object_hold_state/update', 10)
        
        # --- Timers ---
        # Continuously broadcast the current state at 2Hz (every 0.5 seconds)
        # This ensures late-joining nodes immediately know the state without waiting for a change.
        self.create_timer(0.5, self.broadcast_state)
        
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
        """Publish the cached state to the Boolean and String topics."""
        # Publish Boolean
        bool_msg = Bool()
        bool_msg.data = self.is_holding
        self.bool_pub.publish(bool_msg)
        
        # Publish String
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
