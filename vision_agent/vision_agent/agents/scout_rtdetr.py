from ultralytics import RTDETR
import numpy as np
import cv2

class ScoutAgent:
    def __init__(self, model_path):
        print(f"🚀 Loading RT-DETR Global Scout: {model_path}")
        # Use the RTDETR class instead of YOLO
        self.model = RTDETR(model_path)
        self.img_size = 1024  # Matches your training resolution

    def scan(self, frame):
        """Returns detected objects with bounding boxes and simulated segments for agent_node compatibility."""
        # Use trained image size for maximum accuracy
        results = self.model(frame, imgsz=self.img_size, verbose=False)
        detections = []
        
        # Plot standard debug frame for the visualization feed
        debug_frame = results[0].plot()

        for r in results:
            if r.boxes is None:
                continue
                
            for i, box in enumerate(r.boxes):
                # 1. Extract Detection Data
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                # 2. Convert Box to List of Lists for JSON compatibility
                bbox_int = [int(x1), int(y1), int(x2), int(y2)]

                obj = {
                    "label": label,
                    "confidence": round(conf, 3),
                    "box": bbox_int, # Key used in your current processing loop
                    "bbox": bbox_int,
                    "center": [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                    "segments": [] # Default
                }

                # 3. MOCK SEGMENTS FOR COMPATIBILITY
                # Since RT-DETR is detection-only, we provide the 4 corners of the 
                # box as 'segments' so your PCA and Depth Median logic doesn't crash.
                obj["segments"] = [
                    [int(x1), int(y1)], # Top-Left
                    [int(x2), int(y1)], # Top-Right
                    [int(x2), int(y2)], # Bottom-Right
                    [int(x1), int(y2)]  # Bottom-Left
                ]

                detections.append(obj)
                
        return detections, debug_frame