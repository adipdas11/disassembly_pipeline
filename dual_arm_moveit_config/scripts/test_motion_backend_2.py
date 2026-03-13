#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import math
import time

from disassembly_skills.motion_backend import MotionBackend

# ==========================================
# GLOBAL CONFIGURATION
# ==========================================
MOVE_SPEED = 0.1      
HOME_SPEED = 0.2      
GRIPPER_SPEED = 0.3   

WORLD_X = 0.84
WORLD_Y = 0.07
WORLD_Z = 0.95
Z_OFFSET = 0.30       

OPEN_DEG = 35.0
CLOSE_DEG = -35.0 
HOME_DEG = 0.0

LINK_XARM5 = "xarm5_tool0" 
LINK_UF850 = "u1_tool0"    
JOINT_GRIPPER = "rg6_l_out"

HOME_JOINTS = {
    'xarm5_joint1': 0.0, 'xarm5_joint2': 0.0, 'xarm5_joint3': -1.57,
    'xarm5_joint4': 1.57, 'xarm5_joint5': 0.0,
    'u1_joint1': 0.0, 'u1_joint2': 0.0, 'u1_joint3': -1.57,
    'u1_joint4': 0.0, 'u1_joint5': -1.57, 'u1_joint6': 0.0
}
# ==========================================

class SequentialWorldTester(Node):
    def __init__(self):
        super().__init__('sequential_world_tester')
        self.xarm5 = MotionBackend(self, "xarm_arm")
        self.uf850 = MotionBackend(self, "uf_arm")
        self.gripper = MotionBackend(self, "rg6_gripper")
        self.get_logger().info(f"🚀 Robust Synchronized Logic V3 Active")

    def wait_for_gripper(self, target_deg, timeout=5.0):
        """Forces the script to wait until the gripper physically reaches the target."""
        target_rad = math.radians(target_deg)
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout:
            current_pos = self.gripper.current_joint_positions.get(JOINT_GRIPPER, 999)
            if abs(current_pos - target_rad) < 0.05: # 0.05 rad tolerance
                return True
            time.sleep(0.1)
        return False

    def run_test(self):
        try:
            # --- STEP 1: xArm5 ---
            print("\n" + "="*50)
            print("STEP 1: xArm5 Move and Home")
            input("👉 Press ENTER...")
            if self.xarm5.move_to_pose_robust(WORLD_X, WORLD_Y, WORLD_Z + Z_OFFSET, {}, LINK_XARM5, velocity=MOVE_SPEED):
                time.sleep(1.0)
                self.xarm5.move_to_joint_positions(HOME_JOINTS, "xarm5", velocity=HOME_SPEED)
                time.sleep(2.0)

            # --- STEP 2: UF850 Position + Gripper ---
            print("\n" + "="*50)
            print("STEP 2: UF850 Position + Synchronized Gripper")
            input("👉 Press ENTER...")
            
            if self.uf850.move_to_pose_robust(WORLD_X, WORLD_Y, WORLD_Z + Z_OFFSET, {}, LINK_UF850, velocity=MOVE_SPEED):
                # 1. OPEN
                print("Opening Gripper...")
                self.gripper.move_to_joint_positions({JOINT_GRIPPER: math.radians(OPEN_DEG)}, "rg6", velocity=GRIPPER_SPEED)
                self.wait_for_gripper(OPEN_DEG)
                
                # 2. CLOSE
                print("Closing Gripper...")
                self.gripper.move_to_joint_positions({JOINT_GRIPPER: math.radians(CLOSE_DEG)}, "rg6", velocity=GRIPPER_SPEED)
                self.wait_for_gripper(CLOSE_DEG)

                # 3. RESET GRIPPER
                print("Resetting Gripper to 0...")
                self.gripper.move_to_joint_positions({JOINT_GRIPPER: math.radians(HOME_DEG)}, "rg6", velocity=GRIPPER_SPEED)
                self.wait_for_gripper(HOME_DEG)
                
                # 4. UF850 HOME
                time.sleep(1.0) # Safety buffer before arm motion
                print("Returning UF850 to Home...")
                self.uf850.move_to_joint_positions(HOME_JOINTS, "u1", velocity=HOME_SPEED)
                time.sleep(2.0)

            # --- STEP 3: UF850 Full Pose + Return Home ---
            print("\n" + "="*50)
            print("STEP 3: UF850 Full Pose and Mandatory Return")
            input("👉 Press ENTER...")
            
            # Quaternion from captured image
            u1_q_image = {'qx': 0.55531, 'qy': 0.52615, 'qz': 0.46753, 'qw': -0.44295}
            
            # Execute Pose
            success_pose = self.uf850.move_to_pose_robust(WORLD_X, WORLD_Y, WORLD_Z + Z_OFFSET, u1_q_image, LINK_UF850, velocity=MOVE_SPEED)
            
            # Force Home Return regardless of 'success_pose' bool value (to ensure it doesn't skip)
            time.sleep(2.0) 
            print("Final return to Home sequence starting...")
            self.uf850.move_to_joint_positions(HOME_JOINTS, "u1", velocity=HOME_SPEED)
            time.sleep(2.0)

            print("\n✅ Sequence Finished.")

        except Exception as e:
            self.get_logger().error(f"Sequence interrupted: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SequentialWorldTester()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node.run_test()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()