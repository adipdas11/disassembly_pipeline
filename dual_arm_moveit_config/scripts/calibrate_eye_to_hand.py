#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations
import threading
import sys
import time

# ================= CONFIGURATION =================
# CHECKERBOARD SETTINGS
BOARD_DIMS = (10, 7)    # Number of INNER corners (rows, cols)
SQUARE_SIZE = 0.025    # Meters (Measure your printed paper!)

# FRAMES
ROBOT_BASE_FRAME = 'u1_base_link'
ROBOT_EEF_FRAME = 'rg6_hand_tcp'
# =================================================

class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__('hand_eye_calibrator')
        
        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera Subs/Pubs
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.img_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        
        # DEBUG PUBLISHER (For RQT)
        self.debug_pub = self.create_publisher(CompressedImage, '/calibration/preview/compressed', 10)
        
        # State
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_detection = None # Stores (R, t) if board is currently visible
        
        # Storage for calibration
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []

        self.get_logger().info("Ready! Check RQT topic: /calibration/preview/compressed")

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)

    def img_callback(self, msg):
        # 1. Convert ROS Image -> CV2
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        if self.camera_matrix is None:
            return

        # 2. Detect Checkerboard (Live)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, BOARD_DIMS, None)

        # Visuals
        debug_frame = frame.copy()
        
        if found:
            # Refine
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Draw
            cv2.drawChessboardCorners(debug_frame, BOARD_DIMS, corners, found)
            cv2.putText(debug_frame, "PATTERN FOUND", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Calculate Pose for Capture
            objp = np.zeros((BOARD_DIMS[0] * BOARD_DIMS[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:BOARD_DIMS[0], 0:BOARD_DIMS[1]].T.reshape(-1, 2) * SQUARE_SIZE

            ret, rvec, tvec = cv2.solvePnP(objp, corners, self.camera_matrix, self.dist_coeffs)
            
            if ret:
                R_mat, _ = cv2.Rodrigues(rvec)
                self.latest_detection = (R_mat, tvec)
        else:
            self.latest_detection = None
            cv2.putText(debug_frame, "NO PATTERN", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 3. Publish Debug Image
        msg = self.bridge.cv2_to_compressed_imgmsg(debug_frame)
        self.debug_pub.publish(msg)

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(ROBOT_BASE_FRAME, ROBOT_EEF_FRAME, rclpy.time.Time())
            tr = t.transform.translation
            rot = t.transform.rotation
            
            t_vec = np.array([tr.x, tr.y, tr.z]).reshape(3, 1)
            q = [rot.x, rot.y, rot.z, rot.w]
            R_mat = tf_transformations.quaternion_matrix(q)[:3, :3]
            return R_mat, t_vec
        except Exception as e:
            self.get_logger().error(f"TF Error: {e}")
            return None, None

    def capture_sample(self):
        # 1. Check Vision
        if self.latest_detection is None:
            print("❌ Cannot Capture: Checkerboard not visible!")
            return

        # 2. Get Robot Pose
        R_g2b, t_g2b = self.get_robot_pose()
        if R_g2b is None: 
            print("❌ Cannot Capture: TF lookup failed!")
            return

        # 3. Save Data
        R_t2c, t_t2c = self.latest_detection
        
        self.R_gripper2base.append(R_g2b)
        self.t_gripper2base.append(t_g2b)
        self.R_target2cam.append(R_t2c)
        self.t_target2cam.append(t_t2c)
        
        print(f"✅ Sample {len(self.R_gripper2base)} Captured!")

    def compute_calibration(self):
        print("Computing Eye-to-Hand Calibration...")
        
        R_base2gripper = []
        t_base2gripper = []
        
        for R, t in zip(self.R_gripper2base, self.t_gripper2base):
            R_b2g = R.T
            t_b2g = -R_b2g @ t
            R_base2gripper.append(R_b2g)
            t_base2gripper.append(t_b2g)

        try:
            R_cam2base, t_cam2base = cv2.calibrateHandEye(
                R_base2gripper, t_base2gripper,
                self.R_target2cam, self.t_target2cam,
                method=cv2.CALIB_HAND_EYE_TSAI
            )

            R_base2cam = R_cam2base.T
            t_base2cam = -R_base2cam @ t_cam2base

            print("\n=== CALIBRATION RESULT (Copy to URDF) ===")
            print(f"Translation (xyz): [{t_base2cam.flatten()[0]:.4f}, {t_base2cam.flatten()[1]:.4f}, {t_base2cam.flatten()[2]:.4f}]")
            
            homogen = np.eye(4)
            homogen[:3, :3] = R_base2cam
            roll, pitch, yaw = tf_transformations.euler_from_matrix(homogen)
            
            print(f"Rotation (rpy):    [{roll:.4f}, {pitch:.4f}, {yaw:.4f}]")
            print("=========================================")
        except Exception as e:
            print(f"Calibration Failed: {e}")

def spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    calibrator = HandEyeCalibrator()
    
    # Run ROS spinning in background thread so video feed doesn't freeze
    spinner = threading.Thread(target=spin_thread, args=(calibrator,), daemon=True)
    spinner.start()
    
    print("Commands:")
    print("  [ENTER] Capture Sample (Hold robot still!)")
    print("  [c]     Compute & Exit")
    
    try:
        while True:
            i = input("Action: ")
            if i == 'c':
                if len(calibrator.R_gripper2base) < 5:
                    print("Need at least 5 samples!")
                    continue
                calibrator.compute_calibration()
                break
            else:
                calibrator.capture_sample()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()