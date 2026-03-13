#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, Float32, String
from std_srvs.srv import Trigger 
from xarm.wrapper import XArmAPI
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
import time
import math
import threading
import sys
import json

# ================= CONFIGURATION =================
XARM_IP = '192.168.1.239'     
UF850_IP = '192.168.1.195'     
GRIPPER_IP = '192.168.1.1'    

RAD_OPEN = 0.6; RAD_CLOSE = -0.6; RAD_RANGE = RAD_OPEN - RAD_CLOSE
MM_CLOSE = 0.0; MM_OPEN = 160.0      
MAX_RAD_JUMP = 0.4  
GRIPPER_DEFAULT_FORCE_N = 40.0

class RG:
    def __init__(self, gripper, ip, logger, port=502):
        self.logger = logger; self.ip = ip
        self.client = ModbusClient(ip, port=port, stopbits=1, bytesize=8, parity='E', baudrate=115200, timeout=1)
        self.lock = threading.Lock(); self.gripper = gripper
        if self.gripper == 'rg2': self.max_width=1100; self.max_force=400
        elif self.gripper == 'rg6': self.max_width=1600; self.max_force=1200
        
        self.target_force_raw = self.max_force
        self.target_width_mm = 0.0
        self.last_width_mm = 0.0
        self.last_status_raw = 0
        self.client.connect()

    def _clamp_force_raw(self, force_raw):
        return max(0, min(int(force_raw), self.max_force))

    def set_force(self, force_n):
        self.target_force_raw = self._clamp_force_raw(round(float(force_n) * 10.0))
        return self.get_target_force_n()

    def get_target_force_n(self):
        return self.target_force_raw / 10.0
        
    def move_gripper(self, width_mm, force_val=None):
        val = int(width_mm * 10); val = max(0, min(val, self.max_width))
        if force_val is None:
            force_raw = self.target_force_raw
        else:
            force_raw = self._clamp_force_raw(round(float(force_val) * 10.0))
            self.target_force_raw = force_raw
            
        self.target_width_mm = val / 10.0
        with self.lock: 
            try: self.client.write_registers(address=0, values=[force_raw, val, 16], unit=65)
            except: pass
            
    def read_state(self):
        with self.lock: 
            try:
                res = self.client.read_holding_registers(address=267, count=2, unit=65)
                if res and not res.isError():
                    self.last_width_mm = res.registers[0] / 10.0
                    self.last_status_raw = int(res.registers[1])
            except: pass
        return {
            "width_mm": self.last_width_mm,
            "target_width_mm": self.target_width_mm,
            "target_force_n": self.get_target_force_n(),
            "status_raw": self.last_status_raw,
        }

    def get_width(self):
        return self.read_state()["width_mm"]
        
    def close_connection(self):
        with self.lock: self.client.close()

class RealRobotInterface:
    def __init__(self, ip, name, dof, logger, has_linear_track=False):
        self.ip=ip; self.name=name; self.dof=dof; self.logger=logger; self.has_linear_track=has_linear_track
        self.arm = XArmAPI(self.ip); self.connected=False
        
        self.prev_pos = [0.0] * self.dof
        self.prev_time = time.time()
        self.vel_window_size = 5 
        self.vel_history = [[0.0] * self.dof for _ in range(self.vel_window_size)]
        
        # 🛡️ TORQUE FILTER SETTINGS
        self.prev_eff = [0.0] * self.dof
        self.torque_deadband = 0.15   # Ignores jitter below 0.15 N*m
        self.eff_alpha = 0.08         # 0.08 = Strong smoothing for stable signals
        
        self.connect()
    
    def connect(self):
        try:
            self.arm.connect()
            self.arm.motion_enable(enable=True)
            self.arm.clean_error()
            self.arm.set_mode(1) 
            self.arm.set_state(0)
            self.arm.set_report_tau_or_i(1) # Enable SDK torque report
            
            if self.has_linear_track: 
                self.logger.info(f"⚙️ Calibrating {self.name} Linear Track...")
                self.arm.set_linear_track_enable(True)
                self.arm.set_linear_track_back_origin(wait=True) 
                self.arm.set_linear_track_speed(200)
                self.logger.info(f"✅ {self.name} Linear Track homed.")
                
            self.connected=True
        except Exception as e: 
            raise e

    def get_full_state(self):
        if not self.connected: return ([0.0]*self.dof, [0.0]*self.dof, [0.0]*self.dof)
        try:
            now = time.time(); dt = now - self.prev_time
            code_p, pos = self.arm.get_servo_angle(is_radian=True)
            code_t, effort = self.arm.get_joints_torque()
            
            if code_p == 0 and code_t == 0 and pos and effort:
                curr_pos = pos[:self.dof]
                raw_eff = effort[:self.dof]
                
                # --- VELOCITY CALCULATION ---
                curr_vel = [0.0] * self.dof
                if dt > 0.001:
                    inst_vel = [(curr_pos[i] - self.prev_pos[i]) / dt for i in range(self.dof)]
                    self.vel_history.pop(0); self.vel_history.append(inst_vel)
                    for i in range(self.dof):
                        curr_vel[i] = sum(h[i] for h in self.vel_history) / self.vel_window_size
                
                # --- [STAGE 1 & 2: DEADBAND + EMA SMOOTHING] ---
                stable_eff = []
                for i in range(self.dof):
                    raw_val = raw_eff[i] if abs(raw_eff[i]) > self.torque_deadband else 0.0
                    smoothed_val = (self.eff_alpha * raw_val) + ((1 - self.eff_alpha) * self.prev_eff[i])
                    stable_eff.append(round(float(smoothed_val), 3))

                self.prev_pos = curr_pos; self.prev_eff = stable_eff; self.prev_time = now
                return (curr_pos, curr_vel, stable_eff)
        except: pass
        return (self.prev_pos, [0.0]*self.dof, self.prev_eff)

    def reinit_mode(self):
        """Re-applies servo mode and ready state after a web UI reset (fixes SDK code=9)."""
        try:
            self.arm.clean_error()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(1)
            self.arm.set_state(0)
            self.logger.info(f"🔄 {self.name}: Mode re-initialized after web UI reset.")
        except Exception as e:
            self.logger.error(f"❌ {self.name}: reinit_mode failed: {e}")

    def set_servo_angle(self, angles):
        if not self.connected:
            return
        code = self.arm.set_servo_angle_j(angles=angles, is_radian=True)
        if code == 9:
            self.logger.warning(f"⚠️ {self.name}: code=9 detected — robot mode lost (web UI reset?). Re-initializing...")
            self.reinit_mode()

    def set_linear_track(self, pos_meters):
        if self.connected and self.has_linear_track: 
            self.arm.set_linear_track_pos(abs(pos_meters)*1000.0, wait=False)

class RealHardware(Node):
    def __init__(self):
        super().__init__('real_hardware_driver')
        self.cb_group = ReentrantCallbackGroup()
        self.velocity_mode_active = False
        self.last_gripper_cmd = -999.0
        self.initial_sync_complete = False
        
        self.cached_gripper_state = {
            "width_mm": 80.0,
            "target_width_mm": 80.0,
            "target_force_n": GRIPPER_DEFAULT_FORCE_N,
            "status_raw": 0,
        }

        try:
            self.xarm = RealRobotInterface(XARM_IP, "xArm5", 5, self.get_logger(), has_linear_track=True)
            self.uf850 = RealRobotInterface(UF850_IP, "UF850", 6, self.get_logger())
            self.gripper = RG('rg6', GRIPPER_IP, self.get_logger())
            self.gripper.set_force(GRIPPER_DEFAULT_FORCE_N)
        except Exception: 
            self.get_logger().error("❌ HARDWARE INIT FAILED")
            sys.exit(1)

        self.publisher_ = self.create_publisher(JointState, '/robot_joint_states', 50)
        self.gripper_state_pub = self.create_publisher(String, '/rg6/state', 20)
        
        self.cmd_sub = self.create_subscription(JointState, '/robot_joint_commands', self.joint_command_callback, 10, callback_group=self.cb_group)
        self.gripper_force_sub = self.create_subscription(Float32, '/rg6/force_command', self.gripper_force_callback, 10, callback_group=self.cb_group)

        self.loop_count = 0; self.cached_gripper_width = 80.0
        self.perform_blocking_sync()

    def gripper_force_callback(self, msg):
        target_force_n = self.gripper.set_force(msg.data)
        self.cached_gripper_state["target_force_n"] = target_force_n

    def perform_blocking_sync(self):
        self.get_logger().info("⏳ STEP 1: Syncing hardware pose...")
        while rclpy.ok():
            x_p, _, _ = self.xarm.get_full_state()
            if sum([abs(v) for v in x_p]) > 0.01:
                self.get_logger().info("✅ STEP 2: Pose acquired. Flooding...")
                for _ in range(100): 
                    self.publish_real_states()
                    time.sleep(0.01)
                self.initial_sync_complete = True
                self.timer = self.create_timer(0.01, self.publish_real_states, callback_group=self.cb_group) 
                self.get_logger().info("🟢 STEP 3: Driver LIVE. Filtered Torque & Gripper Reporting ACTIVE.")
                return
            time.sleep(0.5)

    def publish_real_states(self):
        try:
            x_p, x_v, x_e = self.xarm.get_full_state()
            u_p, u_v, u_e = self.uf850.get_full_state()
            
            # --- Gripper/Slider tracking ---
            self.loop_count += 1
            if self.gripper and self.loop_count >= 10:
                try: 
                    self.cached_gripper_state = self.gripper.read_state()
                    self.cached_gripper_width = self.cached_gripper_state["width_mm"]
                except: pass
                self.loop_count = 0
                
            g_pos = self.mm_to_rad(self.cached_gripper_width)
            c, s_raw = self.xarm.arm.get_linear_track_pos() if self.xarm else (0, 0.0)
            s_pos = -1.0 * (s_raw / 1000.0) if c == 0 else 0.0
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            # 🛑 MANUAL MAPPING TO MATCH YOUR TOPIC ECHO ORDER
            mapping = [
                ('xarm5_joint1', x_p[0], x_e[0]),
                ('xarm5_joint2', x_p[1], x_e[1]),
                ('slider_slider_joint', s_pos, 0.0),
                ('xarm5_joint5', x_p[4], x_e[4]),
                ('xarm5_joint3', x_p[2], x_e[2]),
                ('u1_joint1',    u_p[0], u_e[0]),
                ('u1_joint2',    u_p[1], u_e[1]),
                ('u1_joint3',    u_p[2], u_e[2]),
                ('xarm5_joint4', x_p[3], x_e[3]),
                ('u1_joint4',    u_p[3], u_e[3]),
                ('u1_joint5',    u_p[4], u_e[4]),
                ('u1_joint6',    u_p[5], u_e[5]),
                ('rg6_l_out',    g_pos, 0.0)
            ]

            msg.name = [m[0] for m in mapping]
            msg.position = [float(m[1]) for m in mapping]
            msg.velocity = [0.0] * len(mapping)

            # 🛡️ THE FINAL ROUNDING FILTER (FORCED)
            clean_efforts = []
            for m in mapping:
                raw_val = float(m[2])
                if abs(raw_val) < 0.15:
                    clean_efforts.append(0.0)
                else:
                    clean_efforts.append(float(f"{raw_val:.3f}"))
            
            msg.effort = clean_efforts
            self.publisher_.publish(msg)
            
            # Publish full JSON state of the gripper
            msg_str = String()
            msg_str.data = json.dumps(self.cached_gripper_state)
            self.gripper_state_pub.publish(msg_str)

        except Exception as e:
            self.get_logger().error(f"Mapping Error: {e}")

    def joint_command_callback(self, msg):
        if self.velocity_mode_active or not self.initial_sync_complete: return
        
        xarm_cmds = [None]*5; uf_cmds = [None]*6; slider_cmd = None; gripper_cmd = None; gripper_force_n = None
        for i, name in enumerate(msg.name):
            if 'xarm5_joint' in name: xarm_cmds[int(name[-1]) - 1] = msg.position[i]
            elif 'u1_joint' in name: uf_cmds[int(name[-1]) - 1] = msg.position[i]
            elif 'slider_slider_joint' == name: slider_cmd = msg.position[i]
            elif 'rg6_l_out' == name: 
                gripper_cmd = msg.position[i]
                if i < len(msg.effort):
                    effort_val = msg.effort[i]
                    if math.isfinite(effort_val) and effort_val > 0.0:
                        gripper_force_n = effort_val

        if all(c is not None for c in xarm_cmds) and self.xarm.connected:
            curr_x, _, _ = self.xarm.get_full_state()
            if all(abs(curr - cmd) < MAX_RAD_JUMP for curr, cmd in zip(curr_x, xarm_cmds)):
                self.xarm.set_servo_angle(xarm_cmds)

        if all(c is not None for c in uf_cmds) and self.uf850.connected:
            curr_u, _, _ = self.uf850.get_full_state()
            if all(abs(curr - cmd) < MAX_RAD_JUMP for curr, cmd in zip(curr_u, uf_cmds)):
                self.uf850.set_servo_angle(uf_cmds)

        if slider_cmd is not None: self.xarm.set_linear_track(slider_cmd)
        
        # Pass force along with position to the updated gripper function
        if gripper_cmd is not None and abs(gripper_cmd - self.last_gripper_cmd) > 0.01:
            self.gripper.move_gripper(self.rad_to_mm(gripper_cmd), force_val=gripper_force_n)
            self.last_gripper_cmd = gripper_cmd

    def rad_to_mm(self, rad): return max(0.0, min(MM_OPEN, ((rad - RAD_CLOSE)/RAD_RANGE)*MM_OPEN))
    def mm_to_rad(self, mm): return ((mm/MM_OPEN)*RAD_RANGE)+RAD_CLOSE

def main(args=None):
    rclpy.init(args=args); node = RealHardware()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()