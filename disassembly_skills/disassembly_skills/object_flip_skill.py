#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
import threading, math, time, os
from disassembly_skills.motion_backend import MotionBackend

HOLD_STATE_FILE = '/tmp/disassembly_hold_state'

def read_hold_state():
    try:
        with open(HOLD_STATE_FILE, 'r') as f:
            return f.read().strip() == 'true'
    except Exception:
        return False

def write_hold_state(held):
    try:
        with open(HOLD_STATE_FILE, 'w') as f:
            f.write('true' if held else 'false')
    except Exception:
        pass

class ObjectFlipSkill(Node):
    def __init__(self):
        super().__init__('object_flip_skill_node')
        
        self.uf850 = MotionBackend(self, "uf_arm")
        self.gripper = MotionBackend(self, "rg6_gripper")
        self.xarm5 = MotionBackend(self, "xarm_arm")
        
        self.state_update_pub = self.create_publisher(String, '/robot_state/manip_arm/update', 10)

        self.PLANNING_FRAME = 'world_world'
        self.ROBOT_EE_LINK = "u1_tool0"
        self.JOINT_GRIPPER = "rg6_l_out"
        self.OPEN_DEG, self.CLOSE_DEG = 35.0, -35.0
        self.RETRACT_Z_HEIGHT = 0.1           
        self.TORQUE_THRESHOLD = 3.0            
        self.DESCENT_SPEED = 0.2              
        self.RETRACT_VELOCITY = 0.5
        self.GRIPPER_OPEN_FORCE_N = 40.0      
        self.GRIPPER_CLOSE_FORCE_N = 100.0     
        
        self.is_holding_object = False
        self.hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hold_status_pub = self.create_publisher(Bool, '/object_hold_state/is_held', self.hold_qos)
        self.create_subscription(Bool, '/object_hold_state/is_held', self.hold_status_callback, self.hold_qos)

        self.get_logger().info("🚀 Object Flip Skill: Compliant with Multi-Arm State Manager.")

    def publish_state(self, s): 
        self.state_update_pub.publish(String(data=s))

    def publish_hold_status(self, h):
        self.hold_status_pub.publish(Bool(data=h))
        write_hold_state(h)

    def hold_status_callback(self, msg):
        self.is_holding_object = msg.data

    def _current_uf_joint_positions(self):
        return {n: p for n, p in self.uf850.current_joint_positions.items() if n.startswith("u1_")}

    def wait_for_arm_settled(self, timeout=10.0):
        start_t = time.time(); settle_timer = 0.0; last_pos = {}
        NOISE_TOLERANCE = 0.006 
        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = self._current_uf_joint_positions()
            if not curr: time.sleep(0.1); continue
            if last_pos:
                max_delta = max([abs(curr[n] - last_pos[n]) for n in curr if n in last_pos], default=0.0)
                if max_delta <= NOISE_TOLERANCE:
                    settle_timer += 0.1
                    if settle_timer >= 0.4: return True
                else: settle_timer = 0.0 
            last_pos = curr; time.sleep(0.1)
        return False

    def wait_for_gripper(self, target_deg, timeout=7.0):
        target_rad = math.radians(target_deg)
        start_t = time.time(); last_pos = 999.0; stall_timer = 0.0; is_closing = target_deg < 0
        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = self.gripper.current_joint_positions.get(self.JOINT_GRIPPER, 999)
            if curr == 999: time.sleep(0.1); continue
            if abs(curr - target_rad) < 0.05: return True
            if abs(curr - last_pos) < 0.002:
                stall_timer += 0.1
                if stall_timer >= 0.8:
                    if is_closing:
                        self.get_logger().info(f"✅ Grasp confirmed at {curr:.3f} rad.")
                        return True
                    else:
                        self.get_logger().warn(f"⚠️ Gripper STUCK while opening. Retrying with high force...")
                        self.gripper.move_to_joint_positions({self.JOINT_GRIPPER: target_rad}, gripper_force_n=100.0)
                        stall_timer = -2.0
            else: stall_timer = 0.0
            last_pos = curr; time.sleep(0.1)
        return False

    def execute_flip(self, interactive=True):
        # Ensure clean state (previous skill may have left servo on a different MotionBackend)
        self.uf850.stop_servo()

        # Wait up to 2 seconds for hold message to arrive (ROS sync buffer)
        sw = time.time()
        while not self.is_holding_object and (time.time() - sw) < 2.0:
            time.sleep(0.1)

        if not self.is_holding_object:
            self.get_logger().error("❌ Error: No object held. Ensure 'object_hold_state/is_held' is publishing True.")
            return False

        self.publish_state("FLIPPING")

        # --- STEP 1: CLOSED-LOOP LIFT ---
        print(f"🚀 STEP 1: Retracting {self.RETRACT_Z_HEIGHT*100}cm (Closed-Loop)...")
        if not self.uf850.start_servo():
            return False
        
        if not self.uf850.retract_servo_z_closed_loop(self.RETRACT_Z_HEIGHT, speed_mps=self.RETRACT_VELOCITY): 
            return False
        self.wait_for_arm_settled()

        # --- STEP 2: FIXED 180° SINGLE ROTATION ---
        print("🔄 STEP 2: Executing Single 180° Flip...")
        joints = self._current_uf_joint_positions()
        current_j6 = joints.get("u1_joint6", 0.0)
        
        # Rotate to the opposite side only once
        if current_j6 > 0:
            print(f"🔄 Rotating -180° from {math.degrees(current_j6):.1f}°...")
            joints["u1_joint6"] = current_j6 - math.pi
        else:
            print(f"🔄 Rotating +180° from {math.degrees(current_j6):.1f}°...")
            joints["u1_joint6"] = current_j6 + math.pi
            
        if not self.uf850.move_to_joint_positions(joints, velocity=0.2): 
            return False
        self.wait_for_arm_settled()

        # --- STEP 3: TACTILE DESCENT ---
        print("⏰ Activating Servo Node for tactile descent...")
        if not self.uf850.start_servo():
            return False

        print(f"⬇️ STEP 3: Tactile Descent (Speed: {self.DESCENT_SPEED}m/s)...")
        # Monitoring Joint 5 (index 4) as requested
        if not self.uf850.move_linear_z_with_torque_stop(self.DESCENT_SPEED, self.TORQUE_THRESHOLD, joint_index=4): 
            return False
        self.wait_for_arm_settled()

        # Stop servo before gripper operations
        self.uf850.stop_servo()

        # --- STEP 4 & 5: RELEASE & RE-GRASP ---
        print("🔓 STEP 4: Releasing...")
        self.publish_state("IDLE")
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: math.radians(self.OPEN_DEG)}, 
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N
        ): return False
        self.wait_for_gripper(self.OPEN_DEG)

        print("🗜️ STEP 5: Re-grasping...")
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: math.radians(self.CLOSE_DEG)}, 
            gripper_force_n=self.GRIPPER_CLOSE_FORCE_N
        ): return False
        self.wait_for_gripper(self.CLOSE_DEG)
        
        self.publish_state("HOLDING")
        self.publish_hold_status(True)
        print("🎉 FLIP COMPLETE")
        return True

def main(args=None):
    rclpy.init(args=args); node = ObjectFlipSkill()
    executor = MultiThreadedExecutor(); executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    # Wait for TF buffer to populate before doing anything
    time.sleep(2.0)
    # Check file-based hold state as fallback (survives process death)
    if not node.is_holding_object and read_hold_state():
        print("📦 Hold state detected from file (previous skill). Proceeding...")
        node.is_holding_object = True
    try:
        if not node.is_holding_object:
            print("❌ No object held. Run object_hold_skill first.")
        else:
            success = node.execute_flip(interactive=False)
            if success:
                print("✅ Flip complete. Broadcasting hold state. Press Ctrl+C to exit.")
                while rclpy.ok():
                    node.publish_hold_status(True)
                    time.sleep(1.0)
            else:
                print("❌ Flip failed.")
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()