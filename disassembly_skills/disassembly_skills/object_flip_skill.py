#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
import threading, math, time
from disassembly_skills.motion_backend import MotionBackend

class ObjectFlipSkill(Node):
    def __init__(self):
        super().__init__('object_flip_skill_node')
        
        self.uf850 = MotionBackend(self, "uf_arm")
        self.gripper = MotionBackend(self, "rg6_gripper")
        self.xarm5 = MotionBackend(self, "xarm_arm")
        
        self.state_update_pub = self.create_publisher(String, '/robot_state/manip_arm/update', 10)
        self.uf_servo_start_client = self.create_client(Trigger, '/uf_servo_node/start_servo')

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
        self.create_subscription(Bool, '/object_hold_state/is_held', self.hold_status_callback, 10)
        
        self.get_logger().info("🚀 Object Flip Skill: Compliant with Multi-Arm State Manager.")

    def publish_state(self, s): 
        self.state_update_pub.publish(String(data=s))

    def hold_status_callback(self, msg): 
        self.is_holding_object = msg.data

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
        if not self._start_uf_servo():
            return False
        time.sleep(1.0)
        
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
        if not self._start_uf_servo():
            return False
        time.sleep(1.0)

        print(f"⬇️ STEP 3: Tactile Descent (Speed: {self.DESCENT_SPEED}m/s)...")
        # Monitoring Joint 5 (index 4) as requested
        if not self.uf850.move_linear_z_with_torque_stop(self.DESCENT_SPEED, self.TORQUE_THRESHOLD, joint_index=4): 
            return False
        self.wait_for_arm_settled()

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
        print("🎉 FLIP COMPLETE")
        return True

def main(args=None):
    rclpy.init(args=args); node = ObjectFlipSkill()
    executor = MultiThreadedExecutor(); executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        while rclpy.ok():
            if node.is_holding_object:
                if node.execute_flip(interactive=True): break
            time.sleep(0.5)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()