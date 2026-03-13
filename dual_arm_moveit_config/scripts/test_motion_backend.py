#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time

# Ensure this matches your package structure
from disassembly_skills.motion_backend import MotionBackend

class JointMotionTester(Node):
    def __init__(self):
        super().__init__('joint_motion_tester')
        
        # Initialize backends with group names matching your SRDF
        self.xarm5 = MotionBackend(self, "xarm_arm")
        self.uf850 = MotionBackend(self, "uf_arm")
        self.dual  = MotionBackend(self, "dual_arms")
        
        self.get_logger().info("✅ Robust Motion Tester Initialized.")
        self.get_logger().info("Supports: Sequential Pose relaxation and Smart Seeding.")

    def run_sequence(self):
        """
        Main test sequence covering both joint-level and pose-level control.
        """
        try:
            # --- TEST 1: xArm5 Robust Pose (Vertical) ---
            print("\n" + "="*40)
            print("TEST 1: Moving xArm5 to Robust Pose (Vertical Point Down)")
            print("Target: xarm5_tool0 at Z=1.1")
            input("👉 Press ENTER to execute...")
            
            # Using relaxation logic (empty q_dict triggers vertical RPY logic)
            success = self.xarm5.move_to_pose_robust(
                x=0.83, y=0.11, z=1.1, 
                q_dict={}, # Triggers RPY (3.14, 0, 0) logic
                link_name="xarm5_tool0"
            )
            self.report_status("xArm5 Pose Move", success)

            # --- TEST 2: UF850 Image-Matched Pose ---
            print("\n" + "="*40)
            print("TEST 2: Moving UF850 to Image-Captured Pose")
            print("Uses strict quaternions from u1_tool0 screenshot")
            input("👉 Press ENTER to execute...")
            
            # Quaternions extracted from image_1ca47b.png
            u1_q = {
                'qx': 0.55531, 'qy': 0.52615, 'qz': 0.46753, 'qw': -0.44295
            }
            success = self.uf850.move_to_pose_robust(
                x=0.82887, y=-0.20629, z=1.0172, 
                q_dict=u1_q, 
                link_name="u1_tool0"
            )
            self.report_status("UF850 Image Pose Move", success)

            # --- TEST 3: Joint Filtering Check ---
            print("\n" + "="*40)
            print("TEST 3: Independent Joint Control (xArm5 J1 ONLY)")
            print("Verifying that UF850 joints stay locked.")
            input("👉 Press ENTER to execute...")
            
            # move_to_joint_positions now includes prefix filtering
            success = self.xarm5.move_to_joint_positions(
                target_joints={'xarm5_joint1': 0.7},
                filter_prefix="xarm5"
            )
            self.report_status("Filtered Joint Move", success)

            # --- TEST 4: Safe Reset ---
            print("\n" + "="*40)
            print("TEST 4: Dual Arm Sequential Reset")
            input("👉 Press ENTER to execute...")
            
            # Precise home positions for disassembly setup
            home_pose = {
                'xarm5_joint1': 0.0, 'xarm5_joint2': 0.0, 'xarm5_joint3': -1.57,
                'xarm5_joint4': 1.57, 'xarm5_joint5': 0.0,
                'u1_joint1': 0.0, 'u1_joint2': 0.0, 'u1_joint3': -1.57,
                'u1_joint4': 0.0, 'u1_joint5': -1.57, 'u1_joint6': 0.0
            }
            # Moving via 'dual' group uses full dictionary
            success = self.dual.move_to_joint_positions(home_pose)
            self.report_status("Global Home Reset", success)

        except Exception as e:
            self.get_logger().error(f"Test sequence interrupted: {e}")

    def report_status(self, task, success):
        if success:
            print(f"✅ {task}: SUCCESS")
        else:
            print(f"❌ {task}: FAILED (Check terminal for IK Error Codes)")

def main(args=None):
    rclpy.init(args=args)
    tester = JointMotionTester()

    executor = MultiThreadedExecutor()
    executor.add_node(tester)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    tester.run_sequence()

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()