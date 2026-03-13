import cv2
import numpy as np

# ==========================================
#   CONFIGURATION: ALL SHAPES (Workspace + Bins)
# ==========================================
# Format: ID: { "TL": (x,y), "TR": (x,y), "BR": (x,y), "BL": (x,y) }
# All coordinates are in mm, relative to the marker center (0,0).

SHAPE_CONFIG = {
    # --- WORKSPACE (ID 0) ---
    # Treats the workspace as one giant shape relative to a single ID 0 marker.
    # Example: Marker is at Top-Left of the table.
    0: {
        "TL": (-10, -175),         # Top-Left is at the marker
        "TR": (-355, -175),       # Table is 600mm wide
        "BR": (-380, 15),     # Table is 600mm wide, 400mm tall
        "BL": (15, 15)        # Table is 400mm tall
    },

    # --- BIN 1 (Top-Left Marker) ---
    1: {
        "TL": (-115, -15),
        "TR": (15, -15),
        "BR": (24, 55),
        "BL": (-112, 55)
    },

    # --- BIN 2 (Center Marker) ---
    2: {
        "TL": (-14, -15),
        "TR": (85, -15),
        "BR": (85, 25),
        "BL": (-17, 25)
    },

    # --- BIN 3 (Top-Right Marker) ---
    3: {
        "TL": (-17, -12),
        "TR": (20, -12),
        "BR": (0, 160),
        "BL": (-45, 160)
    }
}

# System Settings
MARKER_SIZE_MM = 20.0
CAMERA_INDEX = 8 

def run_single_anchor_workspace(camera_index=0):
    print(f"📷 Opening Camera {camera_index}...")
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    if not cap.isOpened():
        print(f"❌ Error: Could not open camera {camera_index}.")
        return

    # ArUco Setup
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    parameters.minMarkerPerimeterRate = 0.02
    parameters.adaptiveThreshWinSizeMax = 35
    parameters.polygonalApproxAccuracyRate = 0.05
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    print("✅ System Started.")
    print("   Workspace is now defined by a SINGLE Anchor (ID 0).")
    print("   Ensure only ONE 'ID 0' marker is visible.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None:
                flat_ids = ids.flatten()
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                for i, marker_id in enumerate(flat_ids):
                    # Check if we have a config for this ID
                    if marker_id in SHAPE_CONFIG:
                        # 1. Calculate Pixels-per-MM
                        perimeter = cv2.arcLength(corners[i][0], True)
                        px_per_mm = (perimeter / 4.0) / MARKER_SIZE_MM

                        # 2. Get Marker Center
                        c = corners[i][0].astype(int)
                        cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))

                        # 3. Get Config & Prepare Points
                        shape_config = SHAPE_CONFIG[marker_id]
                        poly_pts = []
                        
                        # Order: TL -> TR -> BR -> BL
                        for key in ["TL", "TR", "BR", "BL"]:
                            off_x, off_y = shape_config[key]
                            
                            # Apply Offset (MM -> Pixels)
                            pt_x = int(cx + (off_x * px_per_mm))
                            pt_y = int(cy + (off_y * px_per_mm))
                            poly_pts.append([pt_x, pt_y])

                        # 4. Draw The Shape
                        pts_arr = np.array(poly_pts, np.int32).reshape((-1, 1, 2))
                        
                        # Determine Color (Green for Workspace, Yellow for Bins)
                        if marker_id == 0:
                            color = (0, 255, 0)   # Green
                            label = "WORKSPACE"
                            fill_alpha = 0.1
                        else:
                            color = (0, 255, 255) # Yellow
                            label = f"BIN {marker_id}"
                            fill_alpha = 0.2

                        # Draw Outline
                        cv2.polylines(frame, [pts_arr], True, color, 2)
                        
                        # Fill Transparent
                        overlay = frame.copy()
                        cv2.fillPoly(overlay, [pts_arr], color)
                        cv2.addWeighted(overlay, fill_alpha, frame, 1.0 - fill_alpha, 0, frame)
                        
                        # Label (at TL corner)
                        label_pt = poly_pts[0]
                        cv2.putText(frame, label, (label_pt[0], label_pt[1]-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        
                        # Debug: Line from Marker to TL
                        if marker_id == 0:
                             cv2.line(frame, (cx, cy), (poly_pts[0][0], poly_pts[0][1]), (0, 0, 255), 1)

            cv2.imshow('Robust Single-Anchor Workspace', frame)
            if (cv2.waitKey(1) & 0xFF) == ord('q'): break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    run_single_anchor_workspace(camera_index=CAMERA_INDEX)