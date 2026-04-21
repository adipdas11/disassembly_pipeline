#!/usr/bin/env python3
import sys
import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime

# --- SYSTEM PATH INJECTION ---
def find_repo_root(current_path, target_name="disassembly_pipeline"):
    curr = os.path.abspath(current_path)
    while curr != os.path.dirname(curr):
        if os.path.basename(curr) == target_name:
            return curr
        if os.path.exists(os.path.join(curr, target_name)):
            return os.path.join(curr, target_name)
        curr = os.path.dirname(curr)
    return None

WS_ROOT = find_repo_root(__file__)
if WS_ROOT:
    VENV_PATH = os.path.join(WS_ROOT, 'vision_training', '.venv', 'lib', 'python3.10', 'site-packages')
    if os.path.exists(VENV_PATH):
        sys.path.insert(0, VENV_PATH)

class DataCollectorNode(Node):
    def __init__(self):
        super().__init__('data_collector_node')
        self.get_logger().info("--- Camera Data Collection System ---")
        
        self.bridge = CvBridge()
        
        # --- TOPICS (Actual Resolution) ---
        self.global_topic = '/camera/camera/color/image_raw'
        self.local_topic = '/tool_cam/image_raw'
        
        # --- STATE ---
        self.frame_global = None
        self.frame_local = None
        self.is_recording = False
        self.counter = 0
        
        # --- DATA STORAGE ---
        # Save images in vision_training/data_collection/data/session_YYYYMMDD_HHMMSS/
        self.timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_path = os.path.join(WS_ROOT, "vision_training", "data_collection", "data", f"session_{self.timestamp_str}")
        self.global_dir = os.path.join(self.session_path, "global")
        self.local_dir = os.path.join(self.session_path, "local")
        
        os.makedirs(self.global_dir, exist_ok=True)
        os.makedirs(self.local_dir, exist_ok=True)
        
        # --- SUBSCRIBERS ---
        self.sub_global = self.create_subscription(Image, self.global_topic, self.cb_global, 10)
        self.sub_local = self.create_subscription(Image, self.local_topic, self.cb_local, 10)
        
        # --- TIMERS ---
        # One for visual display (~15 FPS is enough for viewer)
        self.display_timer = self.create_timer(1.0/15.0, self.display_callback)
        # One for recording logic (~5 Hz)
        self.record_timer = self.create_timer(0.2, self.record_callback)
        
        self.print_controls()

    def print_controls(self):
        print("\n" + "="*50)
        print("  DATA COLLECTION CONTROLS:")
        print(f"  Saving to: {self.session_path}")
        print("  - [S]: Take Snapshot (Pair of images)")
        print("  - [R]: Toggle Continuous Recording (5 Hz)")
        print("  - [Q]: Quit System")
        print("="*50 + "\n")

    def cb_global(self, msg):
        try:
            self.frame_global = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    def cb_local(self, msg):
        try:
            self.frame_local = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    def save_image_pair(self, mode="record"):
        if self.frame_global is None or self.frame_local is None:
            if mode == "snapshot":
                self.get_logger().warn("Waiting for frames from BOTH cameras...")
            return False
            
        timestamp = datetime.now().strftime("%H%M%S_%f")
        g_name = f"g_{timestamp}.jpg"
        l_name = f"l_{timestamp}.jpg"
        
        cv2.imwrite(os.path.join(self.global_dir, g_name), self.frame_global)
        cv2.imwrite(os.path.join(self.local_dir, l_name), self.frame_local)
        
        self.counter += 1
        return True

    def record_callback(self):
        if self.is_recording:
            self.save_image_pair(mode="record")

    def display_callback(self):
        target_h = 480
        
        # Prepare Global View
        if self.frame_global is not None:
            h, w = self.frame_global.shape[:2]
            scale = target_h / h
            vis_g = cv2.resize(self.frame_global, (int(w * scale), target_h))
        else:
            vis_g = np.zeros((target_h, 640, 3), dtype=np.uint8)
            cv2.putText(vis_g, "WAITING FOR GLOBAL...", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Prepare Local View
        if self.frame_local is not None:
            h, w = self.frame_local.shape[:2]
            scale = target_h / h
            vis_l = cv2.resize(self.frame_local, (int(w * scale), target_h))
        else:
            vis_l = np.zeros((target_h, 640, 3), dtype=np.uint8)
            cv2.putText(vis_l, "WAITING FOR LOCAL...", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
        combined = np.hstack((vis_g, vis_l))
        
        # Add labels & status
        info_panel = np.zeros((60, combined.shape[1], 3), dtype=np.uint8)
        color_rec = (0, 0, 255) if self.is_recording else (0, 255, 0)
        mode_text = "● RECORDING" if self.is_recording else "○ IDLE"
        
        cv2.putText(info_panel, mode_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_rec, 2)
        cv2.putText(info_panel, f"TOTAL PAIRS: {self.counter}", (300, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.putText(info_panel, "[S] SNAP | [R] REC | [Q] QUIT", (combined.shape[1]-450, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        final_frame = np.vstack((combined, info_panel))
        cv2.imshow("Multi-Camera Data Collection", final_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Shutting down...")
            cv2.destroyAllWindows()
            rclpy.shutdown()
            sys.exit(0)
        elif key == ord('s'):
            if self.save_image_pair(mode="snapshot"):
                self.get_logger().info(f"Snapshot #{self.counter} saved.")
        elif key == ord('r'):
            self.is_recording = not self.is_recording
            self.get_logger().info(f"Recording: {'ON' if self.is_recording else 'OFF'}")

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
