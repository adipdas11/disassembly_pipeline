from rfdetr import RFDETRLarge
from PIL import Image
import numpy as np
import cv2

class ScoutAgent:
    def __init__(self, model_path):
        print(f"🚀 Loading Global Scout (RF-DETR): {model_path}")
        self.model = RFDETRLarge(pretrain_weights=model_path, resolution=1088)
        self.model.optimize_for_inference() # Reduce latency
        self.class_names = [
            'Actuator_Arm', 'Connector_Port', 'Exterior_Screw_Zone', 'HDD_Chassis', 
            'Hole', 'Internal_Screw_Zone', 'PCB_Main', 'PCB_Screw_Zone', 
            'Platter', 'Platter_Separator', 'Platter_Separator_Ring', 
            'Spindle_Hub', 'Top_Lid', 'Voice_Coil_Magnet'
        ]

    def scan(self, frame):
        """Returns list of detected objects with bounding boxes and debug image."""
        if frame is None:
            return [], None
            
        # Convert BGR (cv2) to RGB (PIL)
        pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
        # Inference
        detections = self.model.predict(pil_img, threshold=0.4)
        
        processed_detections = []
        debug_frame = frame.copy()
        
        for i in range(len(detections.xyxy)):
            x1, y1, x2, y2 = detections.xyxy[i]
            conf = float(detections.confidence[i])
            cls_id = int(detections.class_id[i])
            label = self.class_names[cls_id] if cls_id < len(self.class_names) else f"Unknown_{cls_id}"
            
            bbox_int = [int(x1), int(y1), int(x2), int(y2)]
            
            obj = {
                "label": label,
                "confidence": round(conf, 3),
                "box": bbox_int,
                "bbox": bbox_int,
                "center": [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                # Mock segments using box corners for compatibility with depth processing logic
                "segments": [
                    [int(x1), int(y1)],
                    [int(x2), int(y1)],
                    [int(x2), int(y2)],
                    [int(x1), int(y2)]
                ]
            }
            processed_detections.append(obj)
            
            # Draw debug visuals
            cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(debug_frame, f"{label} {conf:.2f}", (int(x1), int(y1)-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
        return processed_detections, debug_frame