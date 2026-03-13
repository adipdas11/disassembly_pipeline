#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import serial
import time

class ToolCommander(Node):
    def __init__(self):
        super().__init__('tool_commander')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        # --- Hardware Connection ---
        self.ser = None
        self.fake = False
        
        try:
            # Note: 115200 is standard, but Pico often handles higher or lower.
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            self.get_logger().info(f"Connected to Tool @ {self.port}")
        except (serial.SerialException, OSError):
            self.fake = True
            self.get_logger().warn(f"Could not open {self.port}, running in FAKE mode")

        # --- Subscribers ---
        # 1 = Screw, 0 = Stop, -1 = Unscrew
        # 2 = Grab (155°), 3 = Release (0°)
        self.create_subscription(Int8, 'tool_cmd', self.handle_cmd, 10)

        self.current_cmd = 0

    def handle_cmd(self, msg: Int8):
        cmd = msg.data
        
        # Validating allowed commands: Now including 2 and 3
        if cmd not in [1, 0, -1, 2, 3]:
            self.get_logger().warn(f"Received invalid command: {cmd}")
            return

        self.current_cmd = cmd
        
        # Map IDs to readable names for logging
        cmd_names = {
            1: "SCREW",
            0: "STOP",
            -1: "UNSCREW",
            2: "GRAB",
            3: "RELEASE"
        }
        
        # Send to Hardware
        if not self.fake and self.ser:
            try:
                # Protocol: send the number as string with newline
                payload = f"{cmd}\n".encode('utf-8')
                self.ser.write(payload)
                self.get_logger().info(f"Sent {cmd_names.get(cmd)} command: {cmd}")
            except Exception as e:
                self.get_logger().error(f"Serial Error: {e}")
        else:
            self.get_logger().info(f"[FAKE MODE] Executing {cmd_names.get(cmd)}: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = ToolCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser: 
            try:
                node.ser.write(b"0\n")
            except Exception:
                pass
            try:
                node.ser.close()
            except Exception:
                pass
        try:
            node.destroy_node()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()
