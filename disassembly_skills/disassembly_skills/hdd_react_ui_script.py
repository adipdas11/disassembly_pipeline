#!/usr/bin/env python3
"""
HDD disassembly UI script.

This is a deterministic, ReAct-style operator UI. It does not call an LLM.
The UI shows reasoning, plan, action, and observation text, then waits for
operator approval before executing each backend skill.
"""
import copy
import json
import queue
import threading
import time
import tkinter as tk
from tkinter import scrolledtext

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int8, String

from disassembly_skills.object_flip_drop_skill import FlipDropSkill
from disassembly_skills.object_flip_skill import ObjectFlipSkill
from disassembly_skills.object_hold_skill import ObjectHoldSkill
from disassembly_skills.unscrew_skill import UnscrewSkill


class HDDReactUiNode(Node):
    """ROS node that owns the skill nodes and shared vision state."""

    def __init__(self):
        super().__init__("hdd_react_ui_script")

        self.hold_skill = ObjectHoldSkill()
        self.flip_skill = ObjectFlipSkill()
        self.flip_drop_skill = FlipDropSkill()
        self.unscrew_skill = UnscrewSkill()

        self.vision_lock = threading.Lock()
        self.detected_objects = []
        self.vision_received = threading.Event()
        self.create_subscription(String, "/vision/agent_state", self._vision_cb, 10)
        self.vision_reset_pub = self.create_publisher(String, "/vision/reset_tracker", 10)
        self.tool_stop_pub = self.create_publisher(Int8, "/tool_cmd", 10)

        self.get_logger().info("HDD ReAct UI Script: Ready.")

    def _vision_cb(self, msg):
        try:
            data = json.loads(msg.data.strip().strip("'").strip('"'))
            raw = data.get("global_view", {}).get("objects", [])
            with self.vision_lock:
                self.detected_objects = raw
            self.vision_received.set()
        except Exception as exc:
            self.get_logger().warning(f"Failed to parse /vision/agent_state: {exc}")

    def get_objects(self):
        with self.vision_lock:
            return copy.deepcopy(self.detected_objects)

    def reset_vision(self, settle_sec=2.0):
        self.vision_reset_pub.publish(String(data="reset"))
        time.sleep(settle_sec)

    def find_by_keywords(self, objects, keywords):
        results = []
        for obj in objects:
            label = obj.get("label", "").lower()
            if any(keyword.lower() in label for keyword in keywords):
                results.append(obj)
        return results

    def find_chassis(self):
        return self.find_by_keywords(self.get_objects(), ["hdd", "chassis", "lid"])

    def find_screws(self):
        return self.find_by_keywords(self.get_objects(), ["screw"])

    def stop_all_motion(self):
        """Best-effort stop for operator Stop button."""
        try:
            self.tool_stop_pub.publish(Int8(data=0))
        except Exception:
            pass

        backends = [
            getattr(self.hold_skill, "uf850", None),
            getattr(self.flip_skill, "uf850", None),
            getattr(self.flip_drop_skill, "uf850", None),
            getattr(self.unscrew_skill, "moveit_backend", None),
        ]
        for backend in backends:
            if backend is None:
                continue
            try:
                backend.stop_immediately()
            except Exception:
                pass


class HDDReactUiApp:
    """Tkinter controller for a deterministic ReAct-style HDD workflow."""

    MAX_SCREWS = 2
    AGENT_COLORS = {
        "VISION": {"stage": "#8a1c7c", "header": "#ff5bd8", "body": "#ff9ee8"},
        "REASON": {"stage": "#314f7a", "header": "#8ab4f8", "body": "#b7d0ff"},
        "PLAN": {"stage": "#285f66", "header": "#5eead4", "body": "#9ff4e7"},
        "ACTION": {"stage": "#6d4b1f", "header": "#f59e0b", "body": "#facc6b"},
        "OBSERVE": {"stage": "#2d6a4f", "header": "#74d99f", "body": "#a7f3c1"},
        "ERROR": {"stage": "#8b1e2d", "header": "#ff6b7a", "body": "#ff9aa5"},
    }
    AGENT_NAMES = {
        "VISION": "VISION AGENT",
        "REASON": "REASON AGENT",
        "PLAN": "PLANNER AGENT",
        "ACTION": "ACTION AGENT",
        "OBSERVE": "OBSERVER AGENT",
        "ERROR": "ERROR AGENT",
    }
    TITLE_TO_AGENT = {
        "VISION": "VISION",
        "REASON": "REASON",
        "REASONING": "REASON",
        "PLAN": "PLAN",
        "ACTION": "ACTION",
        "OBSERVE": "OBSERVE",
        "OBSERVATION": "OBSERVE",
        "FINAL": "OBSERVE",
        "ERROR": "ERROR",
    }

    def __init__(self, node: HDDReactUiNode):
        self.node = node
        self.ui_queue = queue.Queue()
        self.current_execute = None
        self.current_skip = None
        self.current_after = None
        self.executing = False
        self.stopped = False
        self.screw_attempts = 0
        self.screws_removed = 0
        self.skipped_screw_ids = set()
        self.phase_token = 0
        self.last_vision_wait_log_t = 0.0
        self.workflow_started = False

        self.root = tk.Tk()
        self.root.title("HDD ReAct Disassembly Control")
        self.root.geometry("980x720")
        self.root.configure(bg="#111315")
        self.root.protocol("WM_DELETE_WINDOW", self.on_stop)

        self._build_ui()
        self._set_status("Waiting for /vision/agent_state...")
        self.root.after(200, self._poll_ros_ready)
        self.root.after(100, self._process_ui_queue)
        self.root.after(300, self._periodic_refresh_objects)

    def _build_ui(self):
        self.stage_frame = tk.Frame(self.root, bg="#111315")
        self.stage_frame.pack(fill=tk.X, padx=16, pady=(16, 8))

        self.stage_labels = {}
        for name in ["VISION", "REASON", "PLAN", "ACTION", "OBSERVE"]:
            label = tk.Label(
                self.stage_frame,
                text=name,
                width=14,
                height=2,
                bg="#2a2f35",
                fg="#b7c0c8",
                font=("DejaVu Sans", 11, "bold"),
                relief=tk.FLAT,
            )
            label.pack(side=tk.LEFT, padx=4)
            self.stage_labels[name] = label

        body = tk.Frame(self.root, bg="#111315")
        body.pack(fill=tk.BOTH, expand=True, padx=16, pady=8)

        left = tk.Frame(body, bg="#171a1e")
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        right = tk.Frame(body, bg="#171a1e", width=280)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(8, 0))
        right.pack_propagate(False)

        self.log = scrolledtext.ScrolledText(
            left,
            wrap=tk.WORD,
            bg="#0b0d0f",
            fg="#e6edf3",
            insertbackground="#e6edf3",
            font=("DejaVu Sans Mono", 10),
            borderwidth=0,
        )
        self.log.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)
        self._configure_log_tags()

        self.status_var = tk.StringVar()
        status = tk.Label(
            right,
            textvariable=self.status_var,
            bg="#171a1e",
            fg="#e6edf3",
            font=("DejaVu Sans", 12, "bold"),
            anchor="w",
            justify=tk.LEFT,
            wraplength=250,
        )
        status.pack(fill=tk.X, padx=12, pady=(12, 8))

        tk.Label(
            right,
            text="Detected objects",
            bg="#171a1e",
            fg="#8b949e",
            font=("DejaVu Sans", 10, "bold"),
            anchor="w",
        ).pack(fill=tk.X, padx=12, pady=(8, 4))

        self.objects_box = tk.Listbox(
            right,
            bg="#0b0d0f",
            fg="#e6edf3",
            selectbackground="#31536f",
            height=14,
            borderwidth=0,
            font=("DejaVu Sans Mono", 9),
        )
        self.objects_box.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

        controls = tk.Frame(right, bg="#171a1e")
        controls.pack(fill=tk.X, padx=12, pady=(0, 12))

        self.execute_btn = tk.Button(
            controls,
            text="Execute",
            command=self.on_execute,
            bg="#2d6a4f",
            fg="white",
            activebackground="#40916c",
            height=2,
            state=tk.DISABLED,
        )
        self.execute_btn.pack(fill=tk.X, pady=4)

        self.skip_btn = tk.Button(
            controls,
            text="Skip",
            command=self.on_skip,
            bg="#5f6c7b",
            fg="white",
            activebackground="#748193",
            height=2,
            state=tk.DISABLED,
        )
        self.skip_btn.pack(fill=tk.X, pady=4)

        self.stop_btn = tk.Button(
            controls,
            text="Stop",
            command=self.on_stop,
            bg="#8b1e2d",
            fg="white",
            activebackground="#b02a3b",
            height=2,
        )
        self.stop_btn.pack(fill=tk.X, pady=4)

    def run(self):
        self.root.mainloop()

    def _set_status(self, text):
        self.status_var.set(text)

    def _set_stage(self, active):
        for name, label in self.stage_labels.items():
            if name == active:
                color = self.AGENT_COLORS.get(name, {}).get("stage", "#2a2f35")
                label.configure(bg=color, fg="white")
            else:
                label.configure(bg="#2a2f35", fg="#b7c0c8")

    def _configure_log_tags(self):
        self.log.tag_configure("default_header", foreground="#e6edf3")
        self.log.tag_configure("default_body", foreground="#c9d1d9")
        for agent, colors in self.AGENT_COLORS.items():
            self.log.tag_configure(
                f"{agent}_header",
                foreground=colors["header"],
                font=("DejaVu Sans Mono", 10, "bold"),
            )
            self.log.tag_configure(
                f"{agent}_body",
                foreground=colors["body"],
                font=("DejaVu Sans Mono", 10),
            )

    def _agent_for_title(self, title):
        return self.TITLE_TO_AGENT.get(title.upper(), "ERROR" if "ERROR" in title.upper() else "OBSERVE")

    def _append(self, title, text):
        agent = self._agent_for_title(title)
        display_title = self.AGENT_NAMES.get(agent, title)
        header_tag = f"{agent}_header" if agent in self.AGENT_COLORS else "default_header"
        body_tag = f"{agent}_body" if agent in self.AGENT_COLORS else "default_body"
        self.log.insert(tk.END, f"[{display_title}] ", header_tag)
        self.log.insert(tk.END, f"{text}\n\n", body_tag)
        self.log.see(tk.END)

    def _refresh_objects(self):
        self.objects_box.delete(0, tk.END)
        for obj in self.node.get_objects():
            xyz = obj.get("xyz")
            z_text = f" z={xyz[2]:.3f}m" if xyz and len(xyz) >= 3 else ""
            conf = obj.get("confidence")
            conf_text = f" conf={conf:.2f}" if isinstance(conf, (int, float)) else ""
            self.objects_box.insert(
                tk.END,
                f"[{obj.get('id', '?')}] {obj.get('label', '?')}{conf_text}{z_text}",
            )

    def _periodic_refresh_objects(self):
        if self.stopped:
            return
        self._refresh_objects()
        self.root.after(500, self._periodic_refresh_objects)

    def _vision_summary(self):
        objects = self.node.get_objects()
        if not objects:
            return "Current /vision/agent_state has no visible objects."

        labels = [
            f"[{obj.get('id', '?')}] {obj.get('label', '?')}"
            for obj in objects[:8]
        ]
        suffix = "" if len(objects) <= 8 else f" (+{len(objects) - 8} more)"
        return f"Current /vision/agent_state: {len(objects)} objects visible: {', '.join(labels)}{suffix}."

    def _poll_ros_ready(self):
        if self.stopped:
            return
        self._refresh_objects()
        if self.workflow_started:
            return

        now = time.time()
        if not self.node.vision_received.is_set():
            if now - self.last_vision_wait_log_t > 2.0:
                self._set_stage("VISION")
                self._append("VISION", "Waiting for /vision/agent_state publisher and first message.")
                self.last_vision_wait_log_t = now
            self.root.after(500, self._poll_ros_ready)
            return

        objects = self.node.get_objects()
        chassis_targets = self.node.find_chassis()
        if chassis_targets:
            self._set_stage("VISION")
            self.workflow_started = True
            labels = ", ".join(
                f"[{obj.get('id', '?')}] {obj.get('label', '?')}"
                for obj in objects[:8]
            )
            self._append(
                "VISION",
                f"Vision ready. {len(objects)} objects detected. Usable hold target: "
                f"[{chassis_targets[0].get('id', '?')}] {chassis_targets[0].get('label', '?')}."
                f" Objects: {labels}",
            )
            self.present_hold()
            return

        if now - self.last_vision_wait_log_t > 2.0:
            labels = ", ".join(
                f"[{obj.get('id', '?')}] {obj.get('label', '?')}"
                for obj in objects[:8]
            ) or "none"
            self._set_stage("VISION")
            self._append(
                "VISION",
                f"Vision is publishing, but no chassis/lid target is available yet. "
                f"Current objects: {labels}",
            )
            self.last_vision_wait_log_t = now
        self.root.after(500, self._poll_ros_ready)

    def _process_ui_queue(self):
        try:
            while True:
                kind, payload = self.ui_queue.get_nowait()
                if kind == "done":
                    result, after_fn = payload
                    self.executing = False
                    self._set_stage("OBSERVE")
                    self._append("OBSERVATION", f"Action returned: {result}")
                    if not self.stopped and after_fn is not None:
                        after_fn(result)
        except queue.Empty:
            pass
        if not self.stopped:
            self.root.after(100, self._process_ui_queue)

    def _enable_action_buttons(self):
        if self.stopped or self.executing:
            return
        self._set_stage("ACTION")
        self._append("ACTION", "Waiting for operator approval. Press Execute, Skip, or Stop.")
        self.execute_btn.configure(state=tk.NORMAL)
        self.skip_btn.configure(state=tk.NORMAL)

    def _run_phase_sequence(self, phases, token, index=0):
        if self.stopped or token != self.phase_token:
            return
        if index >= len(phases):
            self._enable_action_buttons()
            return

        stage, text = phases[index]
        self._set_stage(stage)
        self._append(stage, text)
        self.root.after(850, lambda: self._run_phase_sequence(phases, token, index + 1))

    def present_action(
        self,
        title,
        vision,
        reasoning,
        plan,
        action,
        execute_fn,
        after_fn,
        skip_fn=None,
    ):
        if self.stopped:
            return
        self.phase_token += 1
        self.current_execute = execute_fn
        self.current_after = after_fn
        self.current_skip = skip_fn
        self._set_status(title)
        self._refresh_objects()
        self.execute_btn.configure(state=tk.DISABLED)
        self.skip_btn.configure(state=tk.DISABLED)

        phases = [
            ("VISION", f"{vision}\n{self._vision_summary()}"),
            ("REASON", reasoning),
            ("PLAN", plan),
            ("ACTION", action),
        ]
        self._run_phase_sequence(phases, self.phase_token)

    def on_execute(self):
        if self.executing or self.current_execute is None:
            return
        self.executing = True
        execute_fn = self.current_execute
        after_fn = self.current_after
        self.execute_btn.configure(state=tk.DISABLED)
        self.skip_btn.configure(state=tk.DISABLED)
        self._set_stage("ACTION")
        self._append("ACTION", "Operator approved execution.")

        def worker():
            try:
                result = execute_fn()
            except Exception as exc:
                result = f"ERROR: {exc}"
            self.ui_queue.put(("done", (result, after_fn)))

        threading.Thread(target=worker, daemon=True).start()

    def on_skip(self):
        if self.executing:
            self._append("OBSERVATION", "Cannot skip while an action is executing.")
            return
        self.execute_btn.configure(state=tk.DISABLED)
        self.skip_btn.configure(state=tk.DISABLED)
        self._set_stage("OBSERVE")
        self._append("OBSERVATION", "Operator skipped this action.")
        if self.current_skip is not None:
            self.current_skip()

    def on_stop(self):
        if self.stopped:
            return
        self.stopped = True
        self.execute_btn.configure(state=tk.DISABLED)
        self.skip_btn.configure(state=tk.DISABLED)
        self._set_stage("OBSERVE")
        self._append("OBSERVATION", "Operator requested stop. Sending best-effort motion stop.")
        self.node.stop_all_motion()
        self.root.after(500, self.root.quit)

    def present_hold(self):
        self.present_action(
            title="Hold chassis/lid",
            vision="Use the latest global detections to find the primary HDD body: chassis or lid.",
            reasoning="The chassis or lid must be secured before screw removal or flipping.",
            plan="Use the object hold skill on the best chassis/lid detection.",
            action="hold_object(part_id=<detected chassis/lid id>, label=<detected label>)",
            execute_fn=self._execute_hold,
            after_fn=self._after_hold,
            skip_fn=self.present_unscrew_stage,
        )

    def _execute_hold(self):
        targets = self.node.find_chassis()
        if not targets:
            return False
        target = targets[0]
        return self.node.hold_skill.execute_hold(
            part_id=target.get("id"),
            target_label=target.get("label", "HDD_Chassis"),
            interactive=False,
        )

    def _after_hold(self, result):
        if result is True:
            self.present_unscrew_stage()
            return
        self.present_action(
            title="Hold failed",
            vision="Reuse the latest chassis/lid detections for a controlled retry.",
            reasoning="The hold action did not complete successfully.",
            plan="Retry hold, skip only if the chassis is already physically secured, or stop.",
            action="hold_object(...)",
            execute_fn=self._execute_hold,
            after_fn=self._after_hold,
            skip_fn=self.present_unscrew_stage,
        )

    def present_unscrew_stage(self):
        self.present_action(
            title="Start screw processing",
            vision="Reset and settle vision before selecting screw candidates, so the tracker uses the current front-side scene.",
            reasoning="Visible screw detections should be handled before dumping or flipping the chassis.",
            plan=f"Process up to {self.MAX_SCREWS} screws, asking for approval before each screw.",
            action="scan_screws()",
            execute_fn=self._scan_screws,
            after_fn=lambda _result: self.present_next_screw(),
            skip_fn=self.present_flip_drop,
        )

    def _scan_screws(self):
        self.node.reset_vision(settle_sec=2.0)
        return True

    def present_next_screw(self):
        if self.screw_attempts >= self.MAX_SCREWS:
            self._append("OBSERVATION", f"Screw limit reached: {self.screw_attempts}/{self.MAX_SCREWS}.")
            self.present_flip_drop()
            return

        screws = [
            screw for screw in self.node.find_screws()
            if screw.get("id") not in self.skipped_screw_ids
        ]
        if not screws:
            self._append("OBSERVATION", f"No more unskipped screws found. Removed: {self.screws_removed}.")
            self.present_flip_drop()
            return

        target = screws[0]
        tid = target.get("id")
        label = target.get("label", "screw")
        self.present_action(
            title=f"Unscrew {label} (ID: {tid})",
            vision=f"Selected screw candidate from the current vision list: [{tid}] {label}.",
            reasoning="A screw-like detection is visible and should be handled individually.",
            plan="Run alignment, extraction, and bin drop for this specific screw.",
            action=f"unscrew(unscrew_id={tid}, unscrew_label={label!r})",
            execute_fn=lambda target=target: self._execute_unscrew(target),
            after_fn=self._after_unscrew,
            skip_fn=lambda target=target: self._skip_screw(target),
        )

    def _execute_unscrew(self, target):
        self.screw_attempts += 1
        result = self.node.unscrew_skill.execute_unscrew_command(
            target_id=target.get("id"),
            target_label=target.get("label", "screw"),
            interactive=False,
        )
        if result is True:
            self.screws_removed += 1
        return result

    def _after_unscrew(self, _result):
        self.node.reset_vision(settle_sec=2.0)
        self.present_next_screw()

    def _skip_screw(self, target):
        self.screw_attempts += 1
        self.skipped_screw_ids.add(target.get("id"))
        self.node.reset_vision(settle_sec=1.0)
        self.present_next_screw()

    def present_flip_drop(self):
        self.present_action(
            title="Flip-drop loose parts",
            vision="Front-side screw processing is complete or skipped; vision is now context for a fixed loose-part clearing action.",
            reasoning="After screw handling, loose components can be dumped by flipping in the drop zone.",
            plan="Run flip-drop, then keep the chassis held for the backside flip.",
            action="flip_drop()",
            execute_fn=lambda: self.node.flip_drop_skill.execute_flip_drop(interactive=False),
            after_fn=self._after_flip_drop,
            skip_fn=self.present_flip_back,
        )

    def _after_flip_drop(self, result):
        if result is True:
            self.present_flip_back()
            return
        self.present_action(
            title="Flip-drop failed",
            vision="The scene may have shifted after the failed flip-drop, but this retry still uses the fixed flip-drop routine.",
            reasoning="The flip-drop action did not complete successfully.",
            plan="Retry flip-drop, skip to attempt the backside flip anyway, or stop.",
            action="flip_drop()",
            execute_fn=lambda: self.node.flip_drop_skill.execute_flip_drop(interactive=False),
            after_fn=self._after_flip_drop,
            skip_fn=self.present_flip_back,
        )

    def present_flip_back(self):
        self.present_action(
            title="Flip chassis to back side",
            vision="After loose-part clearing, the chassis should still be the controlled object; flip it to expose the back side.",
            reasoning="The front-side clearing sequence is complete enough to expose the back side.",
            plan="Run the normal flip skill to rotate the held chassis and re-grasp it.",
            action="flip_object()",
            execute_fn=lambda: self.node.flip_skill.execute_flip(interactive=False),
            after_fn=self._after_flip_back,
            skip_fn=self.finish,
        )

    def _after_flip_back(self, result):
        if result is True:
            self.finish()
            return
        self.present_action(
            title="Backside flip failed",
            vision="Backside access was not confirmed; keep the current scene as context before retrying or ending.",
            reasoning="The flip action did not complete successfully.",
            plan="Retry the flip, skip to end the workflow, or stop.",
            action="flip_object()",
            execute_fn=lambda: self.node.flip_skill.execute_flip(interactive=False),
            after_fn=self._after_flip_back,
            skip_fn=self.finish,
        )

    def finish(self):
        self.execute_btn.configure(state=tk.DISABLED)
        self.skip_btn.configure(state=tk.DISABLED)
        self._set_stage("OBSERVE")
        self._set_status("Workflow complete")
        self._append("FINAL", "HDD ReAct UI workflow complete.")


def main(args=None):
    rclpy.init(args=args)
    node = HDDReactUiNode()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    executor.add_node(node.hold_skill)
    executor.add_node(node.flip_skill)
    executor.add_node(node.flip_drop_skill)
    executor.add_node(node.unscrew_skill)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        app = HDDReactUiApp(node)
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
