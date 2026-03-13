from rfdetr import RFDETRSmall
from PIL import Image
import numpy as np
import cv2

class SniperAgent:
    def __init__(self, model_path):
        print(f"🎯 Loading Local Sniper (RF-DETR): {model_path}")
        self.model = RFDETRSmall(pretrain_weights=model_path, resolution=640)
        self.model.optimize_for_inference() # Reduce latency
        self.class_names = ['Hole', 'Screw_Head', 'Tool_Tip']

    def target(self, frame):
        """
        Returns structured dict with screw_heads, tool_tips, and holes.
        """
        if frame is None:
            return {"screw_heads": [], "tool_tips": [], "holes": []}
            
        pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        detections = self.model.predict(pil_img, threshold=0.4)
        
        data = {
            "screw_heads": [],
            "tool_tips": [],
            "holes": []
        }
        
        for i in range(len(detections.xyxy)):
            x1, y1, x2, y2 = detections.xyxy[i]
            conf = float(detections.confidence[i])
            cls_id = int(detections.class_id[i])
            label = self.class_names[cls_id] if cls_id < len(self.class_names) else "Unknown"
            
            box = [int(x1), int(y1), int(x2), int(y2)]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
            obj = {
                "label": label,
                "conf": round(conf, 3),
                "box": box
            }
            
            if label == "Screw_Head":
                obj["center"] = [cx, cy]
                data["screw_heads"].append(obj)
            elif label == "Tool_Tip":
                obj["contact_point"] = [cx, cy]
                data["tool_tips"].append(obj)
            elif label == "Hole":
                data["holes"].append(obj)
                
        return data