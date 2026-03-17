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

class FlipDropSkill(Node):
    def __init__(self):
        super().__init__('flip_drop_skill_node')
        
        # Hardware Backends
        self.uf850 = MotionBackend(self, "uf_arm")
        self.gripper = MotionBackend(self, "rg6_gripper")
        self.xarm5 = MotionBackend(self, "xarm_arm")
        
        # State Tracking
        self.is_holding_object = False
        self.hold_event = threading.Event()
        self.hold_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/object_hold_state/is_held', self.hold_status_callback, self.hold_qos)
        
        # Interfaces
        self.state_update_pub = self.create_publisher(String, '/robot_state/manip_arm/update', 10)
        self.hold_status_pub = self.create_publisher(Bool, '/object_hold_state/is_held', self.hold_qos)
        
        # Configuration
        self.PLANNING_FRAME = 'world_world'
        self.ROBOT_EE_LINK = "u1_tool0"
        self.JOINT_GRIPPER = "rg6_l_out"
        self.OPEN_DEG, self.CLOSE_DEG = 35.0, -35.0
        self.RETRACT_Z_HEIGHT = 0.15           
        self.INTERMEDIATE_POSE = {'x': 0.929872, 'y': -0.633943, 'z': 1.0977}
        self.TORQUE_THRESHOLD = 3.0            
        self.DESCENT_SPEED = 0.2              # Reduced for safety
        self.RETRACT_VELOCITY = 0.5            # Synced with Flip Skill
        self.GRIPPER_CLOSE_FORCE_N = 100.0     # Synced with Flip Skill
        self.GRIPPER_OPEN_FORCE_N = 40.0
        
        self.get_logger().info("🚀 Flip-Drop Skill: Modernized Production Version.")

    def publish_state(self, s): self.state_update_pub.publish(String(data=s))
    def publish_hold_status(self, h):
        self.hold_status_pub.publish(Bool(data=h))
        write_hold_state(h)
    def hold_status_callback(self, msg):
        self.is_holding_object = msg.data
        if self.is_holding_object: self.hold_event.set()

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

    def wait_for_gripper(self, target_deg, timeout=8.0):
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

    def execute_flip_drop(self, interactive=False):
        """Standard Flip-Drop sequence with fixed 180-degree logic."""
        print(f"\n🛠️ [START] Flip-Drop Sequential Task")

        try:
            start_tf = self.uf850.tf_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_EE_LINK, rclpy.time.Time())
            sx, sy, sz = start_tf.transform.translation.x, start_tf.transform.translation.y, start_tf.transform.translation.z
            q_start = {'qx': start_tf.transform.rotation.x, 'qy': start_tf.transform.rotation.y, 
                       'qz': start_tf.transform.rotation.z, 'qw': start_tf.transform.rotation.w}
        except Exception as e:
            self.get_logger().error(f"TF Error: {e}"); return False

        # 1. RETRACT
        print("🚀 STEP 1: Vertical Retract (Closed-Loop)...")
        self.publish_state("FLIPPING")
        if not self.uf850.retract_servo_z_closed_loop(self.RETRACT_Z_HEIGHT, speed_mps=self.RETRACT_VELOCITY): return False
        self.wait_for_arm_settled()

        # 2. TRAVEL
        print("🚛 STEP 2: Traveling to Flip Zone...")
        if not self.uf850.move_to_pose_robust(self.INTERMEDIATE_POSE['x'], self.INTERMEDIATE_POSE['y'], self.INTERMEDIATE_POSE['z'], q_start, velocity=0.1): return False
        self.wait_for_arm_settled()

        # 3. FIXED 180° FLIP
        print("🔄 STEP 3: Executing Single 180° Flip...")
        joints = self.uf850.current_joint_positions.copy()
        orig_j6 = joints.get("u1_joint6", 0.0)
        # Polarity Toggle: Move to opposite pole exactly 180 degrees away
        if orig_j6 > 0: joints["u1_joint6"] = orig_j6 - math.pi
        else: joints["u1_joint6"] = orig_j6 + math.pi
        
        if not self.uf850.move_to_joint_positions(joints): return False
        self.wait_for_arm_settled()
        
        # 4. FLIP BACK
        print("🔄 STEP 4: Resetting Orientation...")
        joints["u1_joint6"] = orig_j6
        if not self.uf850.move_to_joint_positions(joints): return False
        self.wait_for_arm_settled()

        # 5. RETURN
        print("🏠 STEP 5: Returning to Pick XY...")
        if not self.uf850.move_to_pose_robust(sx, sy, sz + self.RETRACT_Z_HEIGHT, q_start, velocity=0.1): return False
        self.wait_for_arm_settled()

        # 6. TACTILE DESCENT
        print("⏰ Activating Servo Node & Descent...")
        if not self.uf850.start_servo(): return False
            
        # Monitoring Joint 5 (index 4) as requested
        if not self.uf850.move_linear_z_with_torque_stop(self.DESCENT_SPEED, self.TORQUE_THRESHOLD, joint_index=4): return False
        self.wait_for_arm_settled()
        self.uf850.jog_cartesian_servo(0.0, 0.0, 0.005, duration=0.5)
        self.wait_for_arm_settled()

        # Stop servo before gripper operations
        self.uf850.stop_servo()

        # 7. RELEASE & RE-GRASP
        print("🔓 STEP 7: Dropping loose parts & Re-grasping...")
        self.publish_state("IDLE")
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: math.radians(self.OPEN_DEG)},
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N
        ): return False
        self.wait_for_gripper(self.OPEN_DEG)

        time.sleep(0.5)  # Let gripper controller clear previous trajectory
        if not self.gripper.move_to_joint_positions(
            {self.JOINT_GRIPPER: math.radians(self.CLOSE_DEG)},
            gripper_force_n=self.GRIPPER_CLOSE_FORCE_N
        ): return False
        success = self.wait_for_gripper(self.CLOSE_DEG)

        if success:
            print("✅ [SUCCESS] Flip-Drop Complete. Chassis Held.")
            self.publish_state("HOLDING")
            self.publish_hold_status(True)
            return True
        return False

def main(args=None):
    rclpy.init(args=args); node = FlipDropSkill()
    executor = MultiThreadedExecutor(); executor.add_node(node)
    
    # Run spin in a separate thread so the main loop can control execution
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    # Wait for TF buffer to populate before doing anything
    time.sleep(2.0)

    # --- SINGLE RUN LOGIC ---
    try:
        print("🕒 Waiting for arm to hold object before starting...")
        # Check file-based hold state as fallback (survives process death)
        if not node.is_holding_object and read_hold_state():
            print("📦 Hold state detected from file (previous skill). Proceeding...")
            node.is_holding_object = True
            node.hold_event.set()

        # Wait until the manager or a previous skill sets the HOLD status
        while rclpy.ok():
            if node.is_holding_object:
                print("📦 Object Hold detected. Starting Flip-Drop...")
                if node.execute_flip_drop(interactive=False):
                    print("🏁 Skill finished successfully. Shutting down.")
                else:
                    print("⚠️ Skill exited with errors. Shutting down.")
                break # 🎯 EXIT THE LOOP AFTER ONE RUN
            time.sleep(0.5)
    except KeyboardInterrupt: pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__': main()