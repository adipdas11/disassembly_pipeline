from ultralytics import YOLO

class RefereeAgent:
    def __init__(self, model_path):
        print(f"⚖️ Loading Referee (Cls): {model_path}")
        self.model = YOLO(model_path)

    def inspect(self, frame):
        """Returns status of the assembly (pass/fail/step_name)."""
        results = self.model(frame, verbose=False)
        
        # Get top prediction
        r = results[0]
        top1_idx = r.probs.top1
        
        status = {
            "state": r.names[top1_idx],
            "confidence": float(r.probs.top1conf)
        }
        return status