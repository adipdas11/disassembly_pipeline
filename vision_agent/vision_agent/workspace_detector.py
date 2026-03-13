#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

# ==========================================
#   CONFIGURATION
# ==========================================
SHAPE_CONFIG = {
    0: {"TL": (-10, -175), "TR": (-355, -175), "BR": (-380, 15), "BL": (15, 15)},
    1: {"TL": (-115, -15), "TR": (15, -15), "BR": (24, 55), "BL": (-112, 55)},
    2: {"TL": (-14, -15), "TR": (85, -15), "BR": (85, 25), "BL": (-17, 25)},
    3: {"TL": (-17, -12), "TR": (20, -12), "BR": (0, 160), "BL": (-45, 160)}
}

class WorkspaceDetector(Node):
    def __init__(self):
        super().__init__('workspace_detector')
        
        # --- Settings ---
        self.marker_size_mm = 20.0
        self.smoothing_window = 15  # Number of frames to average (higher = smoother but slower)
        
        # --- Stability Buffers ---
        # Format: { marker_id: { 'scale': deque(), 'cx': deque(), 'cy': deque() } }
        self.buffers = {} 

        # --- ArUco Setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector_params.minMarkerPerimeterRate = 0.02
        self.detector_params.adaptiveThreshWinSizeMax = 35
        self.detector_params.polygonalApproxAccuracyRate = 0.05
        
        # Use newer Corner Refinement to reduce jitter at the source
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', self.image_callback, 10)
        self.pub_debug = self.create_publisher(Image, '/vision/workspace_debug', 10)
        self.pub_debug_compressed = self.create_publisher(CompressedImage, '/vision/workspace_debug/compressed', 10)
        
        self.get_logger().info("✅ Stabilized Workspace Detector Started")

    def get_smoothed_values(self, m_id, raw_scale, raw_cx, raw_cy):
        """Averages the last N values to remove jitter"""
        # Initialize buffer for this ID if new
        if m_id not in self.buffers:
            self.buffers[m_id] = {
                'scale': deque(maxlen=self.smoothing_window),
                'cx': deque(maxlen=self.smoothing_window),
                'cy': deque(maxlen=self.smoothing_window)
            }
        
        b = self.buffers[m_id]
        
        # Add new values
        b['scale'].append(raw_scale)
        b['cx'].append(raw_cx)
        b['cy'].append(raw_cy)
        
        # Calculate Averages
        avg_scale = sum(b['scale']) / len(b['scale'])
        avg_cx = sum(b['cx']) / len(b['cx'])
        avg_cy = sum(b['cy']) / len(b['cy'])
        
        return avg_scale, avg_cx, avg_cy

    def image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None:
                flat_ids = ids.flatten()
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for i, marker_id in enumerate(flat_ids):
                    if marker_id in SHAPE_CONFIG:
                        # 1. Calculate Raw Values
                        perimeter = cv2.arcLength(corners[i][0], True)
                        raw_px_per_mm = (perimeter / 4.0) / self.marker_size_mm
                        
                        c = corners[i][0].astype(float) # Use float for precision before averaging
                        raw_cx = np.mean(c[:, 0])
                        raw_cy = np.mean(c[:, 1])

                        # 2. APPLY SMOOTHING
                        px_per_mm, cx, cy = self.get_smoothed_values(marker_id, raw_px_per_mm, raw_cx, raw_cy)

                        # 3. Build Shape
                        shape_config = SHAPE_CONFIG[marker_id]
                        poly_pts = []
                        
                        for key in ["TL", "TR", "BR", "BL"]:
                            off_x, off_y = shape_config[key]
                            # Use the SMOOTHED values here
                            pt_x = int(cx + (off_x * px_per_mm))
                            pt_y = int(cy + (off_y * px_per_mm))
                            poly_pts.append([pt_x, pt_y])

                        # 4. Draw
                        pts_arr = np.array(poly_pts, np.int32).reshape((-1, 1, 2))
                        
                        if marker_id == 0:
                            color = (0, 255, 0)
                            label = "WORKSPACE"
                            fill_alpha = 0.1
                        else:
                            color = (0, 255, 255)
                            label = f"BIN {marker_id}"
                            fill_alpha = 0.2

                        cv2.polylines(frame, [pts_arr], True, color, 2)
                        overlay = frame.copy()
                        cv2.fillPoly(overlay, [pts_arr], color)
                        cv2.addWeighted(overlay, fill_alpha, frame, 1.0 - fill_alpha, 0, frame)
                        
                        label_pt = poly_pts[0]
                        cv2.putText(frame, label, (label_pt[0], label_pt[1]-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.pub_debug.publish(out_msg)
            out_msg_compressed = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.pub_debug_compressed.publish(out_msg_compressed)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()