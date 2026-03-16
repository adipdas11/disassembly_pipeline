#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int8
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
import json, time, threading, copy, math
from disassembly_skills.motion_backend import MotionBackend

class UnscrewSkill(Node):
    def __init__(self):
        super().__init__('unscrew_skill_node')
        
        # --- ⚙️ CONFIGURATION SECTION (CORE PARAMETERS) ---
        # Physical Geometry
        self.CONFIG = {
            "TOOL_LENGTH": 0.240,           # m (Screwdriver length)
            "HOVER_DISTANCE": 0.007,        # m (10mm as requested)
            "TRANSIT_LIFT": 0.050,          # m (Safe travel height)
            "REACH_LIMIT": 0.680,           # m (xArm reach radius)
            "MM_PER_PIX": 0.000130,         # m/px (Vision calibration)
            
            # Descent & Alignment
            "XY_SPEED_ALIGN": 0.01,        # m/s (Increased as requested)
            "Z_SPEED_DESCENT": 0.01,       # m/s
            "ALIGN_TOLERANCE_PX": 10.0,      # pixels
            "FORCE_THRESHOLD": 3.0,         # N (Contact detection)

            # Spiral Search
            "SPIRAL_TIMEOUT": 15.0,         # s (Vision fallback)
            "SPIRAL_START_DIST_MM": 5.0,   # mm (First side distance)
            "SPIRAL_GAP_MM": 5.0,           # mm (Increase per 2 sides)
            "SPIRAL_SPEED": 0.015,          # m/s
            
            # Wiggle & Engagement
            "WIGGLE_DIST": 0.002,           # m (3mm as requested for closed-loop)
            "WIGGLE_SPEED": 0.01,          # m/s
            "WIGGLE_FORCE_THRESHOLD": 0.5,  # N (Seating confirmation)
            
            # Extraction
            "EXTRACTION_MAX_TIME": 20.0,    # s
            "EXTRACTION_COMPLIANCE_K": 0.002, # Velocity gain per Newton
            "EXTRACTION_STABLE_TIME": 3.0,  # s
            "POST_GRASP_RETRACT": 0.015,     # m (15mm as requested)
            "RETRACT_SPEED": 0.02,          # m/s
            
            # TF Frames
            "WORLD_FRAME": 'world_world',
            "XARM_BASE_FRAME": 'xarm5_base_link',
            "CAMERA_FRAME": 'camera_color_optical_frame'
        }
        
        # --- Motion Backend ---
        self.moveit_backend = MotionBackend(self, "xarm_arm")
        
        # --- ROS 2 Interfaces ---
        self.vision_sub = self.create_subscription(String, '/vision/agent_state', self.vision_callback, 10)
        self.bin_sub = self.create_subscription(String, '/vision/bin_coordinates', self.bin_callback, 10)
        self.state_pub = self.create_publisher(String, '/robot_state/tool_arm/update', 10)
        self.tool_pub = self.create_publisher(Int8, '/tool_cmd', 10)
        self.xarm_servo_start_client = self.create_client(Trigger, '/xarm_servo_node/start_servo')

        # Thread Safety & State
        self.data_lock = threading.Lock()
        self.latest_targets = []
        self.local_view = {}
        self.cached_bin1_xyz = None  
        
        self.get_logger().info("🚀 Refactored Unscrew Skill Active (Compliance & Safety Updated).")

    # =========================================================================
    # CALLBACKS & HELPERS
    # =========================================================================

    def vision_callback(self, msg):
        try:
            raw_data = msg.data.strip().strip("'").strip('"')
            data = json.loads(raw_data)
            with self.data_lock:
                self.latest_targets = [obj for obj in data.get("global_view", {}).get("objects", []) 
                                     if "screw" in obj.get("label", "").lower()]
                self.local_view = data  
        except: pass

    def bin_callback(self, msg):
        try:
            raw_data = msg.data.strip().strip("'").strip('"')
            data = json.loads(raw_data)
            if "bin_1" in data and "xyz" in data["bin_1"]:
                with self.data_lock:
                    self.cached_bin1_xyz = data["bin_1"]["xyz"]
        except: pass

    def wait_for_arm_settled(self, timeout=20.0):
        """Dynamically monitors joint states to ensures precision moves."""
        start_t = time.time()
        settle_timer = 0.0
        last_positions = {}
        NOISE_TOLERANCE = 0.006 

        while rclpy.ok() and (time.time() - start_t) < timeout:
            curr_positions = self.moveit_backend.current_joint_positions.copy()
            if not curr_positions:
                time.sleep(0.1); continue
                
            if last_positions:
                max_delta = 0.0
                for j_name, j_pos in curr_positions.items():
                    if j_name in last_positions:
                        delta = abs(j_pos - last_positions[j_name])
                        if delta > max_delta: max_delta = delta
                            
                if max_delta <= NOISE_TOLERANCE:
                    settle_timer += 0.1
                    if settle_timer >= 0.4: return True
                else: settle_timer = 0.0 
                    
            last_positions = curr_positions
            time.sleep(0.1)
        return True

    # =========================================================================
    # 1. ALIGNMENT & DESCENT (STAIRCASE)
    # =========================================================================
    def perform_staircase_descent(self):
        print("\n🔍 [DESCENT] Starting Visual Servoing & Force-Controlled Descent...")
        
        # Reset local force baselines
        with self.data_lock:
            force_data = self.local_view.get('force_torque', {}).get('force', {})
            base_fz = force_data.get('z', 0.0)
            
        spiral_idx = 0
        spiral_start_time = None
        retry_count = 0
        MAX_RETRIES = 3
        
        # Smoothing state for slow-vision compensation
        prev_vx = 0.0
        prev_vy = 0.0
        
        while rclpy.ok():
            with self.data_lock:
                local = copy.deepcopy(self.local_view)
            
            # --- FT SENSOR CHECK ---
            current_force = local.get('force_torque', {}).get('force', {})
            diff_fz = abs(current_force.get('z', 0.0) - base_fz)
            
            # --- VISION DATA ---
            local_cam_view = local.get('local_view', {})
            screw = local_cam_view.get('screw_heads', [])
            crosshair = local_cam_view.get('crosshair', [320, 240])

            err_x, err_y, dist_px = 999.0, 999.0, 999.0
            if screw:
                err_x = crosshair[0] - screw[0].get('center', [320, 240])[0]
                err_y = crosshair[1] - screw[0].get('center', [320, 240])[1]
                dist_px = math.hypot(err_x, err_y)

            # --- CONTACT LOGIC ---
            if diff_fz > self.CONFIG["FORCE_THRESHOLD"]:
                print(f"🎯 [CONTACT] Z-Force Contact Detected: {diff_fz:.2f}N.")
                
                # Seating Rotation
                print("🔩 [SEATING] Rotating screwdriver briefly...")
                self.tool_pub.publish(Int8(data=-1)) # Unscrew/Spin
                time.sleep(0.5)
                self.tool_pub.publish(Int8(data=0))
                time.sleep(0.5)

                # Wiggle Test (3mm each direction closed-loop)
                print(f"🔄 [WIGGLE] Verifying seating ({self.CONFIG['WIGGLE_DIST']*1000}mm distance)...")
                with self.data_lock:
                    w_base_fx = self.local_view.get('force_torque', {}).get('force', {}).get('x', 0.0)
                    w_base_fy = self.local_view.get('force_torque', {}).get('force', {}).get('y', 0.0)
                
                # Directions: (dx, dy, axis, name)
                w_directions = [
                    (self.CONFIG["WIGGLE_DIST"], 0.0, 'x', "X+"),
                    (-self.CONFIG["WIGGLE_DIST"], 0.0, 'x', "X-"),
                    (0.0, self.CONFIG["WIGGLE_DIST"], 'y', "Y+"),
                    (0.0, -self.CONFIG["WIGGLE_DIST"], 'y', "Y-")
                ]
                
                success_count = 0
                for dx, dy, axis, name in w_directions:
                    # Move Out Closed-Loop
                    self.moveit_backend.move_servo_xy_closed_loop(dx, dy, speed_mps=self.CONFIG["WIGGLE_SPEED"])
                    time.sleep(0.1)
                    with self.data_lock:
                        curr_f = self.local_view.get('force_torque', {}).get('force', {})
                        spike = abs(curr_f.get(axis, 0.0) - (w_base_fx if axis=='x' else w_base_fy))
                    
                    if spike > self.CONFIG["WIGGLE_FORCE_THRESHOLD"]:
                        print(f"  ✅ {name} SUCCESS | Spike: {spike:.2f}N (Req: {self.CONFIG['WIGGLE_FORCE_THRESHOLD']}N)")
                        success_count += 1
                    else:
                        print(f"  ❌ {name} FAIL    | Spike: {spike:.2f}N (Req: {self.CONFIG['WIGGLE_FORCE_THRESHOLD']}N)")
                    
                    # Return to center Closed-Loop
                    self.moveit_backend.move_servo_xy_closed_loop(-dx, -dy, speed_mps=self.CONFIG["WIGGLE_SPEED"])
                    time.sleep(0.1)

                if success_count >= 3:
                    print(f"✅ [WIGGLE PASS] {success_count}/4 wiggles seated bit.")
                    return True
                else:
                    retry_count += 1
                    print(f"❌ [WIGGLE FAIL] {success_count}/4. Retry {retry_count}/{MAX_RETRIES}...")
                    if retry_count > MAX_RETRIES: return False
                    self.moveit_backend.retract_servo_z_closed_loop(0.010, speed_mps=0.03) # Use closed-loop Z-retract
                    with self.data_lock: base_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)
                    continue

            # --- SEARCH / SERVOING LOGIC ---
            if not screw:
                print(f"⚠️ Vision lost. Starting Closed-Loop Square Spiral Search...")
                
                # Setup side parameters
                side_idx = 0
                side_len_mm = self.CONFIG["SPIRAL_START_DIST_MM"]
                search_speed = self.CONFIG["SPIRAL_SPEED"]
                
                def has_vision():
                    with self.data_lock:
                        v = self.local_view.get('local_view', {})
                        return len(v.get('screw_heads', [])) > 0

                search_start_t = time.time()
                while rclpy.ok() and (time.time() - search_start_t) < self.CONFIG["SPIRAL_TIMEOUT"]:
                    # Map side_idx to World XY displacement
                    # Vision -Y(Up)   -> Robot -X
                    # Vision -X(Left) -> Robot -Y
                    # Vision +Y(Down) -> Robot +X
                    # Vision +X(Right)-> Robot +Y
                    dist_m = side_len_mm / 1000.0
                    dx, dy = 0.0, 0.0
                    dir_name = ""
                    
                    if side_idx % 4 == 0:   dx, dir_name = -dist_m, "-Y (Top)"
                    elif side_idx % 4 == 1: dy, dir_name = -dist_m, "-X (Left)"
                    elif side_idx % 4 == 2: dx, dir_name =  dist_m, "+Y (Bottom)"
                    elif side_idx % 4 == 3: dy, dir_name =  dist_m, "+X (Right)"
                    
                    print(f"  ↗️ Moving Side {dir_name} ({side_len_mm}mm)...")
                    res = self.moveit_backend.move_servo_xy_closed_loop(dx, dy, speed_mps=search_speed, stop_check=has_vision)
                    
                    if res == "STOPPED":
                        print("✅ Vision Regained during spiral. Resuming alignment.")
                        prev_vx, prev_vy = 0.0, 0.0
                        break
                    
                    # Side complete, update for next
                    side_idx += 1
                    if side_idx % 2 == 0:
                        side_len_mm += self.CONFIG["SPIRAL_GAP_MM"]
                
                if (time.time() - search_start_t) >= self.CONFIG["SPIRAL_TIMEOUT"]:
                    print("⚠️ [TIMEOUT] Vision search exceeded limit. Aborting.")
                    return "TIMEOUT"
                
                continue # Re-evaluate 'screw' in the next iteration

            # Clear spiral timing state
            spiral_start_time = None
            
            # Sequential Alignment Logic (Align Y first, then X)
            if abs(err_y) > self.CONFIG["ALIGN_TOLERANCE_PX"]:
                target_vx = (err_y * self.CONFIG["MM_PER_PIX"]) * -1.0 * 2.5
                target_vy = 0.0
                align_mode = "[Y]"
            else:
                target_vx = 0.0
                target_vy = (err_x * self.CONFIG["MM_PER_PIX"]) * -1.0 * 2.5
                align_mode = "[X]"
            
            # Speed Caps
            MAX_XY = self.CONFIG["XY_SPEED_ALIGN"]
            target_vx = max(min(target_vx, MAX_XY), -MAX_XY)
            target_vy = max(min(target_vy, MAX_XY), -MAX_XY)

            # Slow Vision Compensation (Alpha smoothing)
            ALPHA = 0.3
            vx = ALPHA * target_vx + (1.0 - ALPHA) * prev_vx
            vy = ALPHA * target_vy + (1.0 - ALPHA) * prev_vy
            prev_vx, prev_vy = vx, vy

            # Funnel Z Logic (Slow Z if XY error is high)
            z_speed = self.CONFIG["Z_SPEED_DESCENT"]
            if dist_px > 30.0: z_speed = 0.0
            elif dist_px > self.CONFIG["ALIGN_TOLERANCE_PX"]: z_speed *= (self.CONFIG["ALIGN_TOLERANCE_PX"] / dist_px)

            print(f"📉 Descent {align_mode} | ErrX: {err_x:>5.1f} | ErrY: {err_y:>5.1f} | Fz: {diff_fz:.2f}N | Vz: {z_speed*1000:.1f}mm/s")
            self.moveit_backend.jog_cartesian_servo(vx, vy, -z_speed, duration=0.2)
            time.sleep(0.05) 

        return False

    # =========================================================================
    # 2. COMPLIANT EXTRACTION
    # =========================================================================
    def perform_compliant_extraction(self):
        print("\n🔄 [EXTRACTION] Starting Compliant Unthreading & Grasm...")
        
        with self.data_lock:
            baseline_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)
        
        # Start Motor & Gripper (Grab with RG6 command)
        self.tool_pub.publish(Int8(data=-1)) # Unscrew
        time.sleep(0.2)
        self.tool_pub.publish(Int8(data=2))  # Grab
        
        peak_upward_force = 0.0
        last_increase_time = time.time()
        start_time = time.time()
        
        # Compliance Loop
        while rclpy.ok() and (time.time() - start_time) < self.CONFIG["EXTRACTION_MAX_TIME"]:
            with self.data_lock:
                current_fz = self.local_view.get('force_torque', {}).get('force', {}).get('z', 0.0)
            
            upward_force = -(current_fz - baseline_fz) # Negative sensor Z = upward pushing
            
            if upward_force > (peak_upward_force + 1.0):
                peak_upward_force = upward_force
                last_increase_time = time.time()
                
            if (time.time() - last_increase_time) >= self.CONFIG["EXTRACTION_STABLE_TIME"]:
                print(f"🎉 [FREE] Extraction force stabilized at {peak_upward_force:.2f}N.")
                break
            
            # Compliant Speed calculate
            z_speed = max(0.0, min(upward_force * self.CONFIG["EXTRACTION_COMPLIANCE_K"], 0.005))
            self.moveit_backend.jog_cartesian_servo(0.0, 0.0, z_speed, duration=0.1)
            time.sleep(0.05)

        # Stop unscrew motor
        self.tool_pub.publish(Int8(data=0))
        time.sleep(0.5)

        # 15mm Z-Retract after grasp (as requested)
        print(f"⬆️ [POST-GRASP] Retracting {self.CONFIG['POST_GRASP_RETRACT']*1000}mm...")
        self.moveit_backend.retract_servo_z_closed_loop(self.CONFIG["POST_GRASP_RETRACT"], speed_mps=self.CONFIG["RETRACT_SPEED"])
        self.wait_for_arm_settled()
        
        return True

    # =========================================================================
    # MAIN SEQUENCE
    # =========================================================================
    def execute_unscrew_command(self, target_id, target_label, interactive=True):
        print(f"\n🛠️ [START] Unscrew Sequence on ID: {target_id} ({target_label})")
        
        # Verify Bin 1 location
        with self.data_lock: bin1_raw = copy.deepcopy(self.cached_bin1_xyz)
        if not bin1_raw:
            print("❌ Bin 1 location missing. Aborting.")
            return False

        # Find Target Object
        with self.data_lock:
            target_data = next((t for t in self.latest_targets if t['id'] == target_id), None)
        if not target_data or 'xyz' not in target_data:
            print(f"❌ Target {target_id} not found in vision.")
            return False

        # Transforms
        raw_pose = Pose()
        raw_pose.position.x, raw_pose.position.y, raw_pose.position.z = target_data['xyz']
        world_pose = self.moveit_backend.get_transformed_pose(raw_pose, self.CONFIG["CAMERA_FRAME"], self.CONFIG["WORLD_FRAME"])
        base_pose = self.moveit_backend.get_transformed_pose(raw_pose, self.CONFIG["CAMERA_FRAME"], self.CONFIG["XARM_BASE_FRAME"])

        if not world_pose or not base_pose: return False
        
        tx, ty = world_pose.pose.position.x, world_pose.pose.position.y
        dist_base = math.hypot(base_pose.pose.position.x, base_pose.pose.position.y)
        hover_z = world_pose.pose.position.z + self.CONFIG["TOOL_LENGTH"] + self.CONFIG["HOVER_DISTANCE"]

        if dist_base > self.CONFIG["REACH_LIMIT"]:
            print(f"❌ Reach {dist_base:.3f}m exceeds limit.")
            return False

        if interactive: input(f"👉 GATE 1: Approach Hover ({hover_z:.3f}m) [ENTER]")
        
        # Safe lift before transit
        self.moveit_backend.retract_servo_z_closed_loop(self.CONFIG["TRANSIT_LIFT"], speed_mps=0.2)
        self.wait_for_arm_settled()

        # Robust Hover
        if not self.moveit_backend.move_to_pose_robust(tx, ty, hover_z, velocity=0.1):
            print("❌ Approach failed.")
            return False
        self.wait_for_arm_settled()

        if interactive: input("👉 GATE 2: Start Visual Servoing & Descent [ENTER]")
        
        # Activate xArm Servo Node
        if self.xarm_servo_start_client.wait_for_service(timeout_sec=5.0):
            self.xarm_servo_start_client.call_async(Trigger.Request())
        time.sleep(1.0)

        # Stage 1: Descent
        staircase_res = self.perform_staircase_descent()
        if staircase_res == "TIMEOUT" or staircase_res == False:
            print("⚠️ Descent failed or timed out. Lifting to safety.")
            self.moveit_backend.retract_servo_z_closed_loop(0.050, speed_mps=0.05)
            # Proceed to bin to reset or handle next target
            self._navigate_to_bin(bin1_raw)
            return True

        # Stage 2: Extraction
        if interactive: input("👉 GATE 3: Start Compliant Extraction [ENTER]")
        if self.perform_compliant_extraction():
            print("🎉 Screw Extracted.")
            self._navigate_to_bin(bin1_raw)

        return True

    def _navigate_to_bin(self, bin1_raw):
        print("🗑️ [DROP-OFF] Navigating to Bin 1...")
        bin_pose = Pose()
        bin_pose.position.x, bin_pose.position.y, bin_pose.position.z = bin1_raw
        world_bin = self.moveit_backend.get_transformed_pose(bin_pose, self.CONFIG["CAMERA_FRAME"], self.CONFIG["WORLD_FRAME"])
        
        if world_bin:
            bx, by = world_bin.pose.position.x, world_bin.pose.position.y
            bz = world_bin.pose.position.z + self.CONFIG["TOOL_LENGTH"] + 0.050 
            
            if self.moveit_backend.move_to_pose_robust(bx, by, bz, velocity=0.1):
                self.wait_for_arm_settled()
                print("⏬ [RELEASE] Dropping screw...")
                self.tool_pub.publish(Int8(data=3)) # Release
                time.sleep(1.0)
                self.tool_pub.publish(Int8(data=0))
                print("♻️ Reset Complete.")

def main(args=None):
    rclpy.init(args=args)
    node = UnscrewSkill()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        time.sleep(2.0)
        while rclpy.ok():
            target_id, target_label = None, None
            with node.data_lock:
                if node.latest_targets: 
                    target_id = node.latest_targets[0]['id']
                    target_label = node.latest_targets[0].get('label', 'screw')
            
            if target_id is not None:
                node.execute_unscrew_command(target_id, target_label, interactive=True)
                with node.data_lock: node.latest_targets = []
            time.sleep(0.5)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()