#!/usr/bin/env python3
import sys
import os

# Inject virtual environment path for AI models
VENV_PATH = '/home/adip/workspace/disassembly_ws/src/vision_training/.venv/lib/python3.10/site-packages'
sys.path.insert(0, VENV_PATH)

# Silencing warnings
os.environ['NO_ALBUMENTATIONS_UPDATE'] = '1'
os.environ['TRANSFORMERS_VERBOSITY'] = 'error'
os.environ['QT_LOGGING_RULES'] = '*.debug=false;qt.qpa.*=false'


import math 
from collections import deque, OrderedDict

# =====================================================================
# 1. ENVIRONMENT SETUP
# =====================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import WrenchStamped 
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import cv2
import numpy as np
from scipy.spatial import distance as dist
from scipy.optimize import linear_sum_assignment

# =====================================================================
# 2. CONFIGURATION & PARAMETERS
# =====================================================================
DASHBOARD_HEIGHT = 550  
PROCESSING_RATE_HZ = 30.0 

# --- LOCAL VIEW ALIGNMENT SETTINGS ---
LOCAL_CROSSHAIR_OFFSET_X = 25  # pixels
LOCAL_CROSSHAIR_OFFSET_Y = 14   # pixels

# --- IMPORT AGENTS ---
from vision_agent.agents.scout import ScoutAgent
from vision_agent.agents.sniper import SniperAgent
from vision_agent.agents.referee import RefereeAgent

# --- MODEL PATHS ---
PATH_SCOUT = "/home/adip/workspace/disassembly_ws/src/vision_training/Project 1 (Segmentation)/rfdetr/global_model/checkpoint_best_ema.pt"
PATH_SNIPER = "/home/adip/workspace/disassembly_ws/src/vision_training/Project 2 (Tool-Screw)/rfdetr/local_model/checkpoint_best_ema.pt"
PATH_REFEREE = "/home/adip/workspace/disassembly_ws/src/vision_training/Project 3 (Classification)/yolo11/state_model/best.pt"

# --- ARUCO ZONE CONFIGURATION ---
SHAPE_CONFIG = {
    0: {"TL": (-10, -175), "TR": (-355, -175), "BR": (-380, 15), "BL": (15, 15)},
    1: {"TL": (-115, -15), "TR": (15, -15), "BR": (24, 55), "BL": (-112, 55)},
    2: {"TL": (-14, -15), "TR": (85, -15), "BR": (85, 25), "BL": (-17, 25)},
    3: {"TL": (-17, -12), "TR": (20, -12), "BR": (0, 160), "BL": (-45, 160)}
}

# =====================================================================
# 3. GEOMETRY & TRACKING HELPERS
# =====================================================================
def calculate_orientation_pca(pts):
    """Calculates the primary orientation angle of an object using Principal Component Analysis."""
    if pts is None or len(pts) < 3: return 0.0, (0,0), (0,0)
    rect = cv2.minAreaRect(pts)
    (w, h) = rect[1]
    if max(w, h) == 0: return 0.0, (0,0), (0,0)
    if min(w, h) / max(w, h) > 0.85: return None, None, None 

    pts_float = pts.reshape(-1, 2).astype(np.float64)
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(pts_float, mean=None)
    cntr = (int(mean[0,0]), int(mean[0,1]))
    angle_rad = math.atan2(eigenvectors[0,1], eigenvectors[0,0]) 
    angle_deg = math.degrees(angle_rad)
    end_point = (int(cntr[0] + eigenvectors[0,0] * 50), int(cntr[1] + eigenvectors[0,1] * 50))
    return float(angle_deg), cntr, end_point

class AngleStabilizer:
    """Smooths orientation angles over time to prevent jitter."""
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.histories = {} 
    def update(self, obj_id, angle_deg):
        if obj_id not in self.histories: self.histories[obj_id] = deque(maxlen=self.window_size)
        rad = math.radians(angle_deg)
        self.histories[obj_id].append((math.cos(rad), math.sin(rad)))
        avg_cos = sum(v[0] for v in self.histories[obj_id]) / len(self.histories[obj_id])
        avg_sin = sum(v[1] for v in self.histories[obj_id]) / len(self.histories[obj_id])
        return math.degrees(math.atan2(avg_sin, avg_cos))

class Point3DStabilizer:
    """Smooths X, Y, Z coordinates over time to prevent depth sensor jitter."""
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.histories = {} 
    def update(self, obj_id, xyz):
        if xyz is None: return None
        if obj_id not in self.histories: self.histories[obj_id] = deque(maxlen=self.window_size)
        self.histories[obj_id].append(xyz)
        avg_x = sum(p[0] for p in self.histories[obj_id]) / len(self.histories[obj_id])
        avg_y = sum(p[1] for p in self.histories[obj_id]) / len(self.histories[obj_id])
        avg_z = sum(p[2] for p in self.histories[obj_id]) / len(self.histories[obj_id])
        return (round(avg_x, 4), round(avg_y, 4), round(avg_z, 4))

class StaticAnchorTracker:
    """Assigns IDs based on fixed spatial anchors. Includes Deadband locking to eliminate jitter."""
    def __init__(self, tolerance=60, maxDisappeared=3000, deadband=5.0, alpha=0.2):
        self.nextObjectID = 0
        self.anchors = {} 
        self.tolerance = tolerance
        self.maxDisappeared = maxDisappeared
        self.deadband = deadband 
        self.alpha = alpha       

    def update(self, rects, labels):
        assigned_ids = {} 
        
        if len(rects) == 0:
            for obj_id in list(self.anchors.keys()):
                self.anchors[obj_id]['disappeared'] += 1
                if self.anchors[obj_id]['disappeared'] > self.maxDisappeared:
                    del self.anchors[obj_id]
            return assigned_ids

        inputCentroids = []
        for (startX, startY, endX, endY) in rects:
            cX = int((startX + endX) / 2.0)
            cY = int((startY + endY) / 2.0)
            inputCentroids.append((cX, cY))

        used_anchors = set()

        for i, (cx, cy) in enumerate(inputCentroids):
            label = labels[i]
            best_id = None
            min_dist = self.tolerance

            for obj_id, anchor in self.anchors.items():
                if obj_id in used_anchors:
                    continue
                if anchor['label'] != label:
                    continue
                
                dist = math.hypot(cx - anchor['centroid'][0], cy - anchor['centroid'][1])
                if dist < min_dist:
                    min_dist = dist
                    best_id = obj_id

            if best_id is not None:
                used_anchors.add(best_id)
                old_cx, old_cy = self.anchors[best_id]['centroid']
                
                if min_dist < self.deadband:
                    final_cx, final_cy = old_cx, old_cy
                else:
                    final_cx = int(self.alpha * cx + (1 - self.alpha) * old_cx)
                    final_cy = int(self.alpha * cy + (1 - self.alpha) * old_cy)

                assigned_ids[best_id] = (final_cx, final_cy)
                self.anchors[best_id]['centroid'] = (final_cx, final_cy)
                self.anchors[best_id]['disappeared'] = 0
            else:
                new_id = self.nextObjectID
                self.nextObjectID += 1
                self.anchors[new_id] = {'centroid': (cx, cy), 'label': label, 'disappeared': 0}
                assigned_ids[new_id] = (cx, cy)
                used_anchors.add(new_id)

        for obj_id in list(self.anchors.keys()):
            if obj_id not in used_anchors:
                self.anchors[obj_id]['disappeared'] += 1
                if self.anchors[obj_id]['disappeared'] > self.maxDisappeared:
                    del self.anchors[obj_id]

        return assigned_ids

# =====================================================================
# 4. MAIN VISION NODE
# =====================================================================
class AgentNode(Node):
    def __init__(self):
        super().__init__('vision_agent_node')
        self.get_logger().info("--- Vision System (Static Tracker 3000 + Reset Logic) ---")

        # --- LOAD AI MODELS ---
        try:
            self.scout = ScoutAgent(PATH_SCOUT)
            self.sniper = SniperAgent(PATH_SNIPER)
            self.referee = RefereeAgent(PATH_REFEREE)
        except Exception as e:
            self.get_logger().error(f"❌ Model Error: {e}")
            return

        # --- TRACKING & HELPERS ---
        self.tracker = StaticAnchorTracker(tolerance=60, maxDisappeared=3000)
        self.angle_stabilizer = AngleStabilizer(window_size=15)
        self.xyz_stabilizer = Point3DStabilizer(window_size=15)
        self.bridge = CvBridge()
        
        # --- ARUCO SETUP ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX 
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.marker_size_mm = 20.0
        self.smoothing_window = 10 
        self.buffers = {} 
        self.active_polygons = {} 

        # --- STATE VARIABLES ---
        self.frame_global = None
        self.frame_depth_meters = None 
        self.frame_local = None
        self.intrinsics = None 
        self.robot_states = {"tool_arm": "OFFLINE", "manip_arm": "OFFLINE"}
        
        self.wrench_offset = None          
        self.latest_zeroed_wrench = None   
        self.frame_counter = 0 # To decouple Global vs Local frequencies
        self.last_scout_results = ([], None) # Cache for Global scout

        # --- SUBSCRIBERS ---
        self.sub_global = self.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', self.cb_global, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.cb_depth, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.cb_info, 10)
        self.sub_local = self.create_subscription(CompressedImage, '/tool_cam/image_raw/compressed', self.cb_local, 10)
        self.sub_wrench = self.create_subscription(WrenchStamped, '/robotiq_force_torque_sensor_broadcaster/wrench', self.cb_wrench, 10)
        self.sub_states = self.create_subscription(String, '/robot_states', self.cb_robot_states, 10)
        
        # New Reset Topic Subscriber
        self.sub_reset = self.create_subscription(String, '/vision/reset_tracker', self.cb_reset_request, 10)
        
        # --- PUBLISHERS ---
        self.json_pub = self.create_publisher(String, '/vision/agent_state', 10)
        self.bin_pub = self.create_publisher(String, '/vision/bin_coordinates', 10)
        self.debug_pub_raw = self.create_publisher(Image, '/vision/debug_feed', 10)
        self.debug_pub_compressed = self.create_publisher(CompressedImage, '/vision/debug_feed/compressed', 10)
        
        # --- TIMER ---
        self.timer = self.create_timer(1.0 / PROCESSING_RATE_HZ, self.processing_loop)

    # -------------------------------------------------------------------------
    # ROS Callbacks
    # -------------------------------------------------------------------------
    def cb_reset_request(self, msg):
        """Clears all stored tracking IDs and smoothing history."""
        self.get_logger().info("♻️ RESETTING TRACKER AND STABILIZERS...")
        self.tracker.anchors.clear()
        self.tracker.nextObjectID = 0
        self.angle_stabilizer.histories.clear()
        self.xyz_stabilizer.histories.clear()
        self.buffers.clear()
        self.get_logger().info("✅ Reset Complete. Starting fresh IDs.")

    def cb_robot_states(self, msg):
        try:
            self.robot_states = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Robot State Parsing Error: {e}")

    def cb_global(self, msg):
        try: 
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.frame_global = img
        except: pass

    def cb_depth(self, msg):
        try:
            raw_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.frame_depth_meters = raw_depth.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f"Depth Error: {e}")

    def cb_info(self, msg):
        if self.intrinsics is None:
            K = msg.k
            self.intrinsics = {'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5]}
            self.get_logger().info(f"✅ Intrinsics Loaded: fx={K[0]:.1f}, fy={K[4]:.1f}")

    def cb_local(self, msg):
        try: 
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            if img.shape[1] > 640: img = cv2.resize(img, (640, 480))
            self.frame_local = img
        except: pass
        
    def cb_wrench(self, msg):
        if self.wrench_offset is None:
            self.wrench_offset = {
                'fx': msg.wrench.force.x, 'fy': msg.wrench.force.y, 'fz': msg.wrench.force.z,
                'tx': msg.wrench.torque.x, 'ty': msg.wrench.torque.y, 'tz': msg.wrench.torque.z
            }
            self.get_logger().info("✅ Force/Torque Sensor zeroed (Tared)!")
        
        self.latest_zeroed_wrench = {
            "force": {
                "x": msg.wrench.force.x - self.wrench_offset['fx'],
                "y": msg.wrench.force.y - self.wrench_offset['fy'],
                "z": msg.wrench.force.z - self.wrench_offset['fz']
            },
            "torque": {
                "x": msg.wrench.torque.x - self.wrench_offset['tx'],
                "y": msg.wrench.torque.y - self.wrench_offset['ty'],
                "z": msg.wrench.torque.z - self.wrench_offset['tz']
            }
        }

    # -------------------------------------------------------------------------
    # Processing & Visuals
    # -------------------------------------------------------------------------
    def get_smoothed_values(self, m_id, raw_scale, raw_cx, raw_cy):
        if m_id not in self.buffers:
            self.buffers[m_id] = { 'scale': deque(maxlen=self.smoothing_window), 'cx': deque(maxlen=self.smoothing_window), 'cy': deque(maxlen=self.smoothing_window) }
        b = self.buffers[m_id]
        b['scale'].append(raw_scale)
        b['cx'].append(raw_cx)
        b['cy'].append(raw_cy)
        return (sum(b['scale'])/len(b['scale']), sum(b['cx'])/len(b['cx']), sum(b['cy'])/len(b['cy']))

    def get_3d_coordinates(self, cx, cy, segments_pts=None):
        if self.frame_depth_meters is None or self.intrinsics is None:
            return None

        depth_val_m = 0.0
        if segments_pts is not None:
            mask = np.zeros(self.frame_depth_meters.shape, dtype=np.uint8)
            cv2.fillPoly(mask, [segments_pts], 255)
            valid_depths = self.frame_depth_meters[mask == 255]
            valid_depths = valid_depths[valid_depths > 0.001] 
            if len(valid_depths) > 0:
                depth_val_m = float(np.median(valid_depths))
            else:
                return None 
        else:
            h, w = self.frame_depth_meters.shape
            cx, cy = max(0, min(w-1, cx)), max(0, min(h-1, cy))
            depth_val_m = float(self.frame_depth_meters[cy, cx])
            if depth_val_m < 0.001: return None 

        z_m = depth_val_m
        x_m = (cx - self.intrinsics['cx']) * z_m / self.intrinsics['fx']
        y_m = (cy - self.intrinsics['cy']) * z_m / self.intrinsics['fy']
        
        return (round(x_m, 4), round(y_m, 4), round(z_m, 4))

    def draw_wide_dashboard(self, width, objects, bin_locations, status, wrench_data):
        panel = np.zeros((DASHBOARD_HEIGHT, width, 3), dtype=np.uint8)
        def draw_text(img, text, x, y, size=0.8, color=(255, 255, 255), thickness=2):
            cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)

        cv2.rectangle(panel, (0, 0), (width, 45), (40, 40, 40), -1)
        draw_text(panel, "SYSTEM DASHBOARD", 20, 35, 1.0, (0, 255, 255), 2)

        col1_x = 20
        draw_text(panel, "DETECTED PARTS", col1_x, 80, 0.75, (200, 200, 200), 2)
        y = 120
        if objects:
            max_items = 6
            sorted_objects = sorted(objects, key=lambda x: x.get('id', 999))
            for i, obj in enumerate(sorted_objects[:max_items]): 
                label = obj.get('label', 'Unknown')
                obj_id = obj.get('id', '?')
                xyz = obj.get('xyz', None)
                
                display_label = f"#{obj_id}: {label[:10]}"
                if xyz: display_label += f" Z:{xyz[2]:.3f}m"
                else: display_label += " No Depth"
                    
                draw_text(panel, f"> {display_label}", col1_x, y, 0.80, (0, 255, 0), 2)
                y += 35
        else:
            draw_text(panel, "No parts detected", col1_x, y, 0.85, (100, 100, 100), 2)

        col2_x = width // 2 - 120 
        draw_text(panel, "VISION AI STATE", col2_x, 80, 0.75, (200, 200, 200), 2)
        
        state = status['state'].upper()
        box_color = (50, 50, 50)
        if state == "UNSCREWED": box_color = (0, 200, 0)
        elif state == "SCREWED": box_color = (0, 140, 255)
        elif "MISALIGN" in state: box_color = (0, 0, 255)
        
        cv2.rectangle(panel, (col2_x, 100), (col2_x + 350, 150), box_color, -1)
        draw_text(panel, state, col2_x + 20, 135, 0.9, (255, 255, 255), 2)
        
        y = 190
        draw_text(panel, "ROBOT ARM STATES", col2_x, y, 0.75, (200, 200, 200), 2)
        t_state = self.robot_states.get("tool_arm", "OFFLINE")
        m_state = self.robot_states.get("manip_arm", "OFFLINE")
        
        draw_text(panel, f"Tool  (xArm) : {t_state}", col2_x + 10, y + 30, 0.8, (0, 255, 255), 2)
        draw_text(panel, f"Manip (UF850): {m_state}", col2_x + 10, y + 60, 0.8, (0, 255, 255), 2)
        
        y = 290
        if wrench_data:
            fx = wrench_data['force']['x']
            fy = wrench_data['force']['y']
            fz = wrench_data['force']['z']
            tx = wrench_data['torque']['x']
            ty = wrench_data['torque']['y']
            tz = wrench_data['torque']['z']
            
            draw_text(panel, "SENSORS (Zeroed)", col2_x, y, 0.75, (200, 200, 200), 2)
            y += 35
            def f_col(v): return (0, 0, 255) if abs(v) > 50.0 else (0, 255, 0)
            
            draw_text(panel, f"Force X:  {fx:>7.2f} N", col2_x + 10, y, 0.85, f_col(fx), 2)
            draw_text(panel, f"Force Y:  {fy:>7.2f} N", col2_x + 10, y + 35, 0.85, f_col(fy), 2)
            draw_text(panel, f"Force Z:  {fz:>7.2f} N", col2_x + 10, y + 70, 0.85, f_col(fz), 2)

            y += 110
            draw_text(panel, f"Torque X: {tx:>7.3f} Nm", col2_x + 10, y, 0.85, (180, 180, 180), 2)
            draw_text(panel, f"Torque Y: {ty:>7.3f} Nm", col2_x + 10, y + 35, 0.85, (180, 180, 180), 2)
            draw_text(panel, f"Torque Z: {tz:>7.3f} Nm", col2_x + 10, y + 70, 0.85, (180, 180, 180), 2)
        else:
            draw_text(panel, "FT SENSOR OFF", col2_x, y, 0.85, (0, 0, 255), 2)

        col3_x = width - 350
        draw_text(panel, "LOCATIONS (Cam Frame)", col3_x, 80, 0.75, (200, 200, 200), 2)
        y = 120
        
        if "workspace" in bin_locations:
            ws = bin_locations["workspace"]
            xyz = ws.get("xyz")
            if xyz: draw_text(panel, f"WORK: Z:{xyz[2]:.3f}m", col3_x, y, 0.85, (0, 255, 255), 2)
            y += 40

        for i in range(1, 4):
            key = f"bin_{i}"
            if key in bin_locations:
                bin_data = bin_locations[key]
                xyz = bin_data.get("xyz")
                if xyz:
                    draw_text(panel, f"BIN {i}: Z:{xyz[2]:.3f}m", col3_x, y, 0.85, (0, 255, 0), 2)
                else:
                    draw_text(panel, f"BIN {i}: ...", col3_x, y, 0.85, (0, 255, 255), 2)
            else:
                draw_text(panel, f"BIN {i}: NOT FOUND", col3_x, y, 0.85, (0, 0, 255), 2)
            y += 40

        return panel

    def processing_loop(self):
        if self.intrinsics is None:
            self.get_logger().warn("⚠️ Waiting for Camera Intrinsics...", throttle_duration_sec=2.0)
        elif self.frame_depth_meters is None:
            self.get_logger().warn("⚠️ Waiting for Depth Image...", throttle_duration_sec=2.0)

        if self.frame_global is None and self.frame_local is None: return 
        timestamp = self.get_clock().now().nanoseconds
        objects = []
        bin_locations = {} 
        vis_global = None
        
        if self.frame_global is not None:
            # OPTIMIZATION: Run Global Scout at 5Hz (every 6th frame if loop is 30Hz)
            if self.frame_counter % 6 == 0:
                self.last_scout_results = self.scout.scan(self.frame_global)
            
            detections, vis_global = self.last_scout_results
            if vis_global is None: vis_global = self.frame_global.copy()
            else: vis_global = vis_global.copy() # Use the latest cached debug image
            
            overlay = vis_global.copy()
            gray = cv2.cvtColor(vis_global, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None:
                flat_ids = ids.flatten()
                for i, marker_id in enumerate(flat_ids):
                    if marker_id in SHAPE_CONFIG:
                        perimeter = cv2.arcLength(corners[i][0], True)
                        raw_px_per_mm = (perimeter / 4.0) / self.marker_size_mm
                        c = corners[i][0].astype(float)
                        raw_cx, raw_cy = np.mean(c[:, 0]), np.mean(c[:, 1])
                        px_per_mm, cx, cy = self.get_smoothed_values(marker_id, raw_px_per_mm, raw_cx, raw_cy)
                        
                        poly_pts = []
                        for key in ["TL", "TR", "BR", "BL"]:
                            off_x, off_y = SHAPE_CONFIG[marker_id][key]
                            poly_pts.append([int(cx + (off_x * px_per_mm)), int(cy + (off_y * px_per_mm))])
                        
                        poly_arr = np.array(poly_pts, np.int32).reshape((-1, 1, 2))
                        self.active_polygons[marker_id] = poly_arr

                        xyz_meters = self.get_3d_coordinates(int(cx), int(cy), poly_arr)
                        key_name = "workspace" if marker_id == 0 else f"bin_{marker_id}"
                        
                        bin_locations[key_name] = {
                            "id": int(marker_id),
                            "px": [int(cx), int(cy)],
                            "xyz": xyz_meters 
                        }

            workspace_poly = self.active_polygons.get(0, None)
            for m_id, poly in self.active_polygons.items():
                color = (0, 255, 0) if m_id == 0 else (0, 255, 255)
                cv2.polylines(vis_global, [poly], True, color, 2)
                cv2.putText(vis_global, "WORKSPACE" if m_id==0 else f"BIN {m_id}", tuple(poly[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            raw_objects, _ = self.scout.scan(self.frame_global)
            raw_objects.sort(key=lambda x: x.get('box', [0])[0])
            
            valid_objects = []
            rects = []
            labels = [] 
            
            for obj in raw_objects:
                box = obj.get('box') or obj.get('bbox') or obj.get('xyxy')
                label = obj.get('label') 
                if not box: continue
                
                raw_cx = int((box[0] + box[2]) / 2)
                raw_cy = int((box[1] + box[3]) / 2)
                is_valid = False
                if workspace_poly is not None and cv2.pointPolygonTest(workspace_poly, (raw_cx, raw_cy), False) >= 0:
                    is_valid = True
                
                if is_valid:
                    valid_objects.append(obj)
                    rects.append(box) 
                    labels.append(label) 
                else:
                    cv2.rectangle(vis_global, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 0, 255), 1)

            tracked_objects = self.tracker.update(rects, labels)
            
            for obj in valid_objects:
                box = obj.get('box') or obj.get('bbox') or obj.get('xyxy')
                raw_cx = int((box[0] + box[2]) / 2)
                raw_cy = int((box[1] + box[3]) / 2)
                
                obj_id = -1
                min_dist = 9999
                for t_id, t_center in tracked_objects.items():
                    d = np.sqrt((raw_cx - t_center[0])**2 + (raw_cy - t_center[1])**2)
                    if d < min_dist and d < 50:
                        min_dist = d
                        obj_id = t_id
                obj['id'] = obj_id 
                
                if obj_id != -1 and obj_id in tracked_objects:
                    cx, cy = tracked_objects[obj_id]
                else:
                    cx, cy = raw_cx, raw_cy
                
                segments = obj.get('segments') or obj.get('mask')
                angle = 0.0
                pca_center = (cx, cy)
                
                pts = None
                if segments:
                    pts = np.array(segments, np.int32).reshape((-1, 1, 2))
                    raw_angle, pca_center, axis_end = calculate_orientation_pca(pts)
                    if raw_angle is not None:
                        angle = self.angle_stabilizer.update(obj_id, raw_angle)
                    elif obj_id in self.angle_stabilizer.histories:
                         h = self.angle_stabilizer.histories[obj_id]
                         avg_cos = sum(v[0] for v in h)/len(h)
                         avg_sin = sum(v[1] for v in h)/len(h)
                         angle = math.degrees(math.atan2(avg_sin, avg_cos))

                xyz_meters = self.get_3d_coordinates(cx, cy, pts) 
                
                if xyz_meters and obj_id != -1:
                    xyz_meters = self.xyz_stabilizer.update(obj_id, xyz_meters)

                if xyz_meters: obj['xyz'] = xyz_meters 
                
                obj['angle'] = angle 
                objects.append(obj)

                color = (0, 255, 0)
                if segments:
                    cv2.fillPoly(overlay, [pts], color)
                    cv2.polylines(vis_global, [pts], True, color, 2)
                    if raw_angle is not None:
                         rad = math.radians(angle)
                         smooth_end = (int(cx + math.cos(rad)*50), int(cy + math.sin(rad)*50))
                         cv2.line(vis_global, (cx, cy), smooth_end, (0, 0, 255), 3)
                else:
                    cv2.rectangle(overlay, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), color, -1)
                    cv2.rectangle(vis_global, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), color, 2)
                
                info_text = f"ID:{obj_id}"
                if xyz_meters: info_text += f" Z:{xyz_meters[2]:.2f}m"
                cv2.putText(vis_global, info_text, (int(box[0]), int(box[1])-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
            cv2.addWeighted(overlay, 0.3, vis_global, 0.7, 0, vis_global)
        else:
            vis_global = np.zeros((480, 640, 3), dtype=np.uint8)

        sniper_data = {"screw_heads": [], "tool_tips": [], "holes": [], "crosshair": []}
        status = {"state": "unknown", "confidence": 0.0}
        vis_local = None

        if self.frame_local is not None:
            vis_local = self.frame_local.copy()
            h_loc, w_loc = vis_local.shape[:2]

            raw_sniper_data = self.sniper.target(self.frame_local)
            sniper_data.update(raw_sniper_data) 
            status = self.referee.inspect(self.frame_local)
            
            # --- LOCAL VISUALIZATION OVERHAUL: BULLETPROOF CENTER CALCULATION ---
            for s in sniper_data['screw_heads']:
                if "box" in s:
                    box = s["box"]
                    cx = int((box[0] + box[2]) / 2)
                    cy = int((box[1] + box[3]) / 2)
                    
                    # 1. PUSH TO DICTIONARY (Ensures it goes to JSON)
                    s["center"] = [cx, cy]
                    
                    # 2. Draw Visuals
                    cv2.rectangle(vis_local, (box[0], box[1]), (box[2], box[3]), (255, 255, 0), 2)
                    cv2.putText(vis_local, "Screw", (box[0], box[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    cv2.circle(vis_local, (cx, cy), 5, (255, 255, 0), -1) 
            
            for t in sniper_data['tool_tips']:
                if "box" in t:
                    box = t["box"]
                    cx = int((box[0] + box[2]) / 2)
                    cy = int((box[1] + box[3]) / 2)
                    
                    # 1. PUSH TO DICTIONARY (Ensures it goes to JSON)
                    t["contact_point"] = [cx, cy]
                    
                    # 2. Draw Visuals
                    cv2.circle(vis_local, (cx, cy), 5, (255, 0, 255), -1)
                    cv2.putText(vis_local, "Tool", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                elif "contact_point" in t: # Fallback in case old code runs
                    cx, cy = t["contact_point"]
                    cv2.circle(vis_local, (int(cx), int(cy)), 5, (255, 0, 255), -1)
                    cv2.putText(vis_local, "Tool", (int(cx)+10, int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            
            for h in sniper_data['holes']:
                if "box" in h:
                    box = h["box"]
                    cx = int((box[0] + box[2]) / 2)
                    cy = int((box[1] + box[3]) / 2)
                    
                    # 1. PUSH TO DICTIONARY (Ensures it goes to JSON)
                    h["center"] = [cx, cy]
                    
                    # 2. Draw Visuals
                    cv2.rectangle(vis_local, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
                    cv2.putText(vis_local, "Hole", (box[0], box[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    cv2.circle(vis_local, (cx, cy), 3, (0, 0, 255), -1)

            # --- CROSSHAIR ---
            cross_x = (w_loc // 2) + LOCAL_CROSSHAIR_OFFSET_X
            cross_y = (h_loc // 2) + LOCAL_CROSSHAIR_OFFSET_Y
            sniper_data["crosshair"] = [int(cross_x), int(cross_y)] 
            
            cv2.line(vis_local, (cross_x - 20, cross_y), (cross_x + 20, cross_y), (0, 255, 0), 2)
            cv2.line(vis_local, (cross_x, cross_y - 20), (cross_x, cross_y + 20), (0, 255, 0), 2)
            cv2.circle(vis_local, (cross_x, cross_y), 2, (0, 0, 255), -1)

            # Axis Lines
            ax_org_x, ax_org_y = w_loc - 60, 40
            cv2.arrowedLine(vis_local, (ax_org_x, ax_org_y), (ax_org_x + 30, ax_org_y), (0, 0, 255), 2, tipLength=0.3)
            cv2.putText(vis_local, "X", (ax_org_x + 35, ax_org_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.arrowedLine(vis_local, (ax_org_x, ax_org_y), (ax_org_x, ax_org_y + 30), (0, 255, 0), 2, tipLength=0.3)
            cv2.putText(vis_local, "Y", (ax_org_x - 5, ax_org_y + 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            vis_local = np.zeros((480, 640, 3), dtype=np.uint8)

        wrench_dict = self.latest_zeroed_wrench if self.latest_zeroed_wrench else {}

        packet = {
            "timestamp": timestamp,
            "global_view": {"objects": objects},
            "local_view": sniper_data, 
            "assembly_state": status,
            "force_torque": wrench_dict 
        }
        self.json_pub.publish(String(data=json.dumps(packet)))
        
        if bin_locations: 
            self.bin_pub.publish(String(data=json.dumps(bin_locations)))

        try:
            h_target = 480 
            def resize_h(img, target_h):
                h, w = img.shape[:2]
                scale = target_h / h
                return cv2.resize(img, (int(w * scale), target_h))
            
            viz_g = resize_h(vis_global, h_target)
            viz_l = resize_h(vis_local, h_target)
            top_row = np.hstack((viz_g, viz_l))
            
            dashboard = self.draw_wide_dashboard(top_row.shape[1], objects, bin_locations, status, self.latest_zeroed_wrench)
            final_frame = np.vstack((top_row, dashboard))
            cv2.putText(final_frame, "GLOBAL (RGB+D)", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            cv2.putText(final_frame, "TOOL CAMERA", (viz_g.shape[1]+20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            
            self.debug_pub_compressed.publish(self.bridge.cv2_to_compressed_imgmsg(final_frame))
            
            raw_msg = self.bridge.cv2_to_imgmsg(final_frame, "bgr8")
            raw_msg.header.stamp = self.get_clock().now().to_msg()
            raw_msg.header.frame_id = "vision_debug"
            self.debug_pub_raw.publish(raw_msg)
            
        except Exception as e:
            self.get_logger().error(f"Vis Error: {e}")
        
        self.frame_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()