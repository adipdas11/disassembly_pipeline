#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
import json, time, threading, math
from disassembly_skills.motion_backend import MotionBackend

class PickupSkill(Node):
    def __init__(self):
        super().__init__('pickup_skill_node')
        # Motion Backends
        self.uf850 = MotionBackend(self, "uf_arm")
        self.gripper = MotionBackend(self, "rg6_gripper")
        self.xarm5 = MotionBackend(self, "xarm_arm")
        
        # Interfaces
        self.state_update_pub = self.create_publisher(String, '/robot_state/manip_arm/update', 10)
        self.uf_servo_start_client = self.create_client(Trigger, '/uf_servo_node/start_servo')
        self.vision_reset_pub = self.create_publisher(String, '/vision/reset_tracker', 10)
        self.hold_status_pub = self.create_publisher(Bool, '/object_hold_status', 10)
        
        # Physical Parameters
        self.UF_TOOL_LENGTH = 0.26
        self.HOVER_HEIGHT = 0.05    
        self.DESCENT_SPEED = 0.09              
        self.TORQUE_THRESHOLD = 3.0            
        self.RETRACT_DIST = 0.05               
        self.RETRACT_VELOCITY = 0.5
        self.POST_GRASP_RETRACT_SPEED = 0.01   # New variable for slow, safe lifts
        self.OPEN_DEG, self.CLOSE_DEG = 33.0, -35.0
        self.GRIPPER_CLOSE_FORCE_N = 50.0      # Updated to 30N as requested
        self.GRIPPER_OPEN_FORCE_N = 20.0
        
        self.UF_HOME_JOINTS = {'u1_joint1': 0.0, 'u1_joint2': 0.0, 'u1_joint3': -1.57, 'u1_joint4': 0.0, 'u1_joint5': -1.57, 'u1_joint6': 0.0}
        self.DROP_POSE = {'x': 0.92, 'y': -0.36, 'z': 1.25}

        # Thread Safety & State
        self.data_lock = threading.Lock()
        self.latest_targets = []
        self.is_holding_object = False
        
        # Subscriptions
        self.create_subscription(Bool, '/object_hold_state/is_held', self.hold_status_callback, 10)
        self.create_subscription(String, '/vision/agent_state', self.vision_callback, 10)
        
    def hold_status_callback(self, msg): 
        self.is_holding_object = msg.data

    def vision_callback(self, msg):
        try:
            data = json.loads(msg.data.strip().strip("'").strip('"'))
            with self.data_lock: self.latest_targets = data.get("global_view", {}).get("objects", [])
        except: pass

    def publish_state(self, s): 
        self.state_update_pub.publish(String(data=s))

    def _start_uf_servo(self, timeout_sec=2.0):
        if not self.uf_servo_start_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("❌ /uf_servo_node/start_servo is unavailable.")
            return False
        future = self.uf_servo_start_client.call_async(Trigger.Request())
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.time() >= deadline: return False
            time.sleep(0.01)
        response = future.result()
        return response is not None and response.success

    def _current_uf_joint_positions(self):
        return {n: p for n, p in self.uf850.current_joint_positions.items() if n.startswith("u1_")}

    def wait_for_arm_settled(self, backend=None, timeout=10.0):
        if backend is None: backend = self.uf850
        start_t = time.time(); settle_timer = 0.0; last_pos = {}
        NOISE_TOLERANCE = 0.006 
        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr = {n: p for n, p in backend.current_joint_positions.items()}
            if not curr: time.sleep(0.1); continue
            if last_pos:
                max_delta = max([abs(curr[n] - last_pos[n]) for n in curr if n in last_pos], default=0.0)
                if max_delta <= NOISE_TOLERANCE:
                    settle_timer += 0.1
                    if settle_timer >= 0.4: return True
                else: settle_timer = 0.0 
            last_pos = curr; time.sleep(0.1)
        return False

    def execute_pickup(self, target_id, target_label):
        print(f"\n🛠️ [START] {target_label} Sequence (ID: {target_id})")

        # --- STEP 0: PRE-CONDITION ---
        if self.is_holding_object:
            print("📦 [PRE-CONDITION] Active Hold Detected. Releasing...")
            self.publish_state("IDLE")
            if not self.gripper.move_to_joint_positions(
                {"rg6_l_out": math.radians(self.OPEN_DEG)},
                gripper_force_n=self.GRIPPER_OPEN_FORCE_N
            ): return False
            time.sleep(1.0)
            self.hold_status_pub.publish(Bool(data=False))
            
            print("⬆️ Vertical Retract (30cm)...")
            if not self.uf850.retract_relative_z(0.30, velocity=self.RETRACT_VELOCITY):
                print("❌ Retract failed. Aborting.")
                return False
            self.wait_for_arm_settled()

            print("🏠 Homing UF850...")
            if not self.uf850.move_to_joint_positions(self.UF_HOME_JOINTS, velocity=self.RETRACT_VELOCITY): 
                return False
            self.wait_for_arm_settled()

        # --- STEP 1: CLEAR WORKSPACE ---
        print("🏠 Clearing xArm5 workspace...")
        if not self.xarm5.move_to_joint_positions({'xarm5_joint1': 0.0, 'xarm5_joint2': 0.0, 'xarm5_joint3': -1.57, 'xarm5_joint4': 1.57, 'xarm5_joint5': 0.0}):
            return False
        self.wait_for_arm_settled(self.xarm5)

        # --- STEP 2: COORDINATE TRANSFORM ---
        with self.data_lock: target = next((t for t in self.latest_targets if t['id'] == target_id), None)
        if not target: return False
        
        raw_p = Pose()
        raw_p.position.x, raw_p.position.y, raw_p.position.z = target['xyz']
        world_p = self.uf850.get_transformed_pose(raw_p, 'camera_color_optical_frame', 'world_world')
        if not world_p: return False

        tx, ty = world_p.pose.position.x - 0.02, world_p.pose.position.y + 0.018
        final_z = world_p.pose.position.z + self.UF_TOOL_LENGTH
        hover_z = final_z + self.HOVER_HEIGHT

        # --- STEP 3: APPROACH & CLOSED-LOOP DESCENT ---
        print("🔓 Opening Gripper for Approach...")
        self.publish_state("MOVING")
        if not self.gripper.move_to_joint_positions(
            {"rg6_l_out": math.radians(self.OPEN_DEG)},
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N
        ): return False
        
        print(f"🚁 Hovering at {tx:.3f}, {ty:.3f}...")
        if not self.uf850.move_to_pose_robust(tx, ty, hover_z, velocity=0.1): return False
        self.wait_for_arm_settled()

        print("🗜️ Closing Gripper to 0 radians for search...")
        if not self.gripper.move_to_joint_positions(
            {"rg6_l_out": 0.0},
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N
        ): return False
        time.sleep(1.0)

        print(f"⬇️ Tactile Descent until force spike (Speed: {self.DESCENT_SPEED}m/s)...")
        if not self._start_uf_servo(): return False
        # Uses the logic in motion_backend where contact is spike > (threshold * 5)
        if not self.uf850.move_linear_z_with_torque_stop(self.DESCENT_SPEED, self.TORQUE_THRESHOLD): 
            return False
        self.wait_for_arm_settled()

        print("⬆️ Retracting 10mm after contact...")
        if not self._start_uf_servo(): return False
        if not self.uf850.retract_servo_z_closed_loop(0.01, speed_mps=self.POST_GRASP_RETRACT_SPEED): return False
        self.wait_for_arm_settled()

        # --- STEP 4: GRASP & RETRACT ---
        print(f"🗜️ Final Grasp at {self.GRIPPER_CLOSE_FORCE_N}N...")
        if not self.gripper.move_to_joint_positions(
            {"rg6_l_out": math.radians(self.CLOSE_DEG)},
            gripper_force_n=self.GRIPPER_CLOSE_FORCE_N
        ): return False
        time.sleep(1.5)
        self.publish_state("HOLDING")
        self.hold_status_pub.publish(Bool(data=True))

        print("⬆️ Final Retract 30mm...")
        if not self._start_uf_servo(): return False
        if not self.uf850.retract_servo_z_closed_loop(0.03, speed_mps=self.POST_GRASP_RETRACT_SPEED): return False
        self.wait_for_arm_settled()

        # --- STEP 5: DROP-OFF ---
        print("🗑️ Moving to Drop Pose...")
        if not self.uf850.move_to_pose_robust(self.DROP_POSE['x'], self.DROP_POSE['y'], self.DROP_POSE['z'], velocity=0.1): return False
        self.wait_for_arm_settled()

        print("🎉 Finalizing: Release & Home...")
        self.publish_state("IDLE")
        if not self.gripper.move_to_joint_positions(
            {"rg6_l_out": math.radians(self.OPEN_DEG)},
            gripper_force_n=self.GRIPPER_OPEN_FORCE_N
        ): return False
        time.sleep(1.0); self.hold_status_pub.publish(Bool(data=False))
        
        if not self.uf850.move_to_joint_positions(self.UF_HOME_JOINTS, velocity=self.RETRACT_VELOCITY): return False
        
        print("✅ [SUCCESS] Sequence Complete.")
        return True

def main(args=None):
    rclpy.init(args=args); node = PickupSkill()
    executor = MultiThreadedExecutor(); executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(1.0); node.vision_reset_pub.publish(String(data='reset'))
    try:
        while rclpy.ok():
            tid = None
            with node.data_lock:
                pcbs = [t for t in node.latest_targets if 'pcb_main' in t.get('label', '').lower()]
                if pcbs: tid = pcbs[0]['id']
            if tid and node.execute_pickup(tid, "pcb_main"): break
            time.sleep(0.5)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()