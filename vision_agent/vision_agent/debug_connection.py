#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data # <--- The "Nuclear Option" for connection

class ConnectionTest(Node):
    def __init__(self):
        super().__init__('connection_test')
        
        # NOTE: Check if your topic has one '/camera' or two!
        # I am using the one you gave me previously:
        topic_name = '/camera/camera/color/image_raw/compressed' 

        self.get_logger().info(f"🔎 Attempting to connect to: {topic_name}")
        self.get_logger().info("⏳ Waiting for data...")

        self.sub = self.create_subscription(
            CompressedImage,
            topic_name,
            self.callback,
            qos_profile_sensor_data # Auto-matches "Best Effort"
        )

    def callback(self, msg):
        # If this prints, your connection works!
        self.get_logger().info(f"✅ SUCCESS! Received frame. Size: {len(msg.data)} bytes")

def main(args=None):
    rclpy.init(args=args)
    node = ConnectionTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()