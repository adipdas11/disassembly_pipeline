#!/usr/bin/env python3
"""
HDD Disassembly Script — Pure state-machine, no LLM.

Flow:
  1. Subscribe to vision, wait for detections
  2. Ask operator before each major action
  3. Hold the HDD chassis
  4. Unscrew up to 2 screws
  5. Flip-drop loose parts
  6. Flip to expose the back side
  7. Done
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
import json, time, threading, math, copy

from disassembly_skills.object_hold_skill import ObjectHoldSkill
from disassembly_skills.object_flip_skill import ObjectFlipSkill
from disassembly_skills.object_flip_drop_skill import FlipDropSkill
from disassembly_skills.unscrew_skill import UnscrewSkill


class HDDDisassemblyScript(Node):
    def __init__(self):
        super().__init__('hdd_disassembly_script')

        # ── Skill nodes (added to executor separately) ──────────────────
        self.hold_skill    = ObjectHoldSkill()
        self.flip_skill    = ObjectFlipSkill()
        self.flip_drop_skill = FlipDropSkill()
        self.unscrew_skill = UnscrewSkill()

        # ── Vision state ────────────────────────────────────────────────
        self.vision_lock     = threading.Lock()
        self.detected_objects = []
        self.vision_received = threading.Event()
        self.create_subscription(
            String, '/vision/agent_state', self._vision_cb, 10
        )

        # ── Vision reset publisher ──────────────────────────────────────
        self.vision_reset_pub = self.create_publisher(
            String, '/vision/reset_tracker', 10
        )

        self.get_logger().info("HDD Disassembly Script: Ready.")

    def _vision_cb(self, msg):
        try:
            data = json.loads(msg.data.strip().strip("'").strip('"'))
            raw = data.get("global_view", {}).get("objects", [])
            with self.vision_lock:
                self.detected_objects = raw
            self.vision_received.set()
        except Exception:
            pass

    def get_objects(self):
        """Return a snapshot of current detections."""
        with self.vision_lock:
            return copy.deepcopy(self.detected_objects)

    def wait_for_vision(self, timeout=10.0):
        """Block until at least one vision frame arrives."""
        self.vision_received.clear()
        return self.vision_received.wait(timeout=timeout)

    def find_by_keywords(self, objects, keywords):
        """Return all objects whose label contains ANY keyword (case-insensitive)."""
        results = []
        for o in objects:
            label = o.get("label", "").lower()
            if any(kw.lower() in label for kw in keywords):
                results.append(o)
        return results

    def find_chassis(self, objects):
        return self.find_by_keywords(objects, ["hdd", "chassis", "lid"])

    def find_screws(self, objects):
        return self.find_by_keywords(objects, ["screw"])

    def operator_gate(self, step_name, next_action, allow_skip=True):
        """Ask the operator whether to run, skip, or stop before a major step."""
        print("\n" + "-" * 60)
        print(f"NEXT ACTION: {step_name}")
        print(f"  {next_action}")
        if not allow_skip:
            prompt = "Press Enter/r to run, or q to quit: "
            valid = {"", "r", "run", "q", "quit", "exit"}
            invalid_msg = "  Invalid input. Use Enter/r or q."
        else:
            prompt = "Press Enter/r to run, s to skip, or q to quit: "
            valid = {"", "r", "run", "s", "skip", "q", "quit", "exit"}
            invalid_msg = "  Invalid input. Use Enter/r, s, or q."

        while rclpy.ok():
            choice = input(prompt).strip().lower()
            if choice in valid:
                if choice in {"q", "quit", "exit"}:
                    return "quit"
                if choice in {"s", "skip"}:
                    print(f"  Skipping: {step_name}")
                    return "skip"
                return "run"
            print(invalid_msg)
        return "quit"

    def run_or_skip(self, step_name, next_action, step_fn, allow_skip=True):
        decision = self.operator_gate(step_name, next_action, allow_skip=allow_skip)
        if decision == "quit":
            print("\nSTOP: Operator requested stop.")
            return "quit", None
        if decision == "skip":
            return "skip", None
        return "run", step_fn()

    def step_hold_chassis(self):
        """Find and hold the HDD chassis or top lid."""
        print("\n" + "=" * 60)
        print("STEP: HOLD CHASSIS / LID")
        print("=" * 60)

        for attempt in range(3):
            self.wait_for_vision()
            objects = self.get_objects()
            chassis_list = self.find_chassis(objects)

            if not chassis_list:
                print(f"  No HDD chassis/lid detected (attempt {attempt+1}/3). Waiting...")
                time.sleep(2.0)
                continue

            target = chassis_list[0]
            tid, tlabel = target.get("id"), target.get("label", "unknown")
            print(f"  Found: {tlabel} (ID: {tid})")

            success = self.hold_skill.execute_hold(
                part_id=tid, target_label=tlabel, interactive=False
            )
            if success:
                print("  Hold successful.")
                return True
            else:
                print("  Hold failed. Retrying...")
                time.sleep(1.0)

        print("  Could not hold chassis after 3 attempts.")
        return False

    def step_unscrew(self, max_screws=2, interactive=True):
        """Unscrew up to max_screws visible screws."""
        print("\n" + "=" * 60)
        print(f"STEP: UNSCREW (max {max_screws})")
        print("=" * 60)

        screws_removed = 0
        skipped_screw_ids = set()
        for i in range(max_screws):
            self.vision_reset_pub.publish(String(data='reset'))
            time.sleep(2.0)
            self.wait_for_vision()
            objects = self.get_objects()
            screws = [
                screw for screw in self.find_screws(objects)
                if screw.get("id") not in skipped_screw_ids
            ]

            if not screws:
                print(f"  No screws detected. {screws_removed} removed total.")
                break

            target = screws[0]
            tid, tlabel = target.get("id"), target.get("label", "screw")
            print(f"  Candidate screw: {tlabel} (ID: {tid}) — screw {i+1}/{max_screws}")

            if interactive:
                decision = self.operator_gate(
                    f"Unscrew {tlabel} (ID: {tid})",
                    "Run the xArm screwdriver alignment, extraction, and bin drop for this screw.",
                )
                if decision == "quit":
                    print("\nSTOP: Operator requested stop.")
                    return "quit"
                if decision == "skip":
                    skipped_screw_ids.add(tid)
                    print(f"  Skipped screw ID {tid}. Moving to next screw check.")
                    continue

            print(f"  Unscrewing: {tlabel} (ID: {tid}) — screw {i+1}/{max_screws}")

            success = self.unscrew_skill.execute_unscrew_command(
                target_id=tid, target_label=tlabel, interactive=False
            )
            if success:
                screws_removed += 1
                print(f"  Screw {screws_removed} removed.")
            else:
                print(f"  Unscrew failed on screw {i+1}. Continuing...")

        print(f"  Unscrew step complete: {screws_removed}/{max_screws} removed.")
        return screws_removed > 0

    def step_flip_drop(self):
        """Flip/drop loose parts, then re-grasp the chassis."""
        print("\n" + "=" * 60)
        print("STEP: FLIP-DROP LOOSE PARTS")
        print("=" * 60)
        return self.flip_drop_skill.execute_flip_drop(interactive=False)

    def run(self, interactive=True):
        print("\n" + "#" * 60)
        print("#  HDD DISASSEMBLY SCRIPT — START")
        print("#" * 60)

        # ── Wait for first vision frame ─────────────────────────────────
        print("\nWaiting for vision data...")
        while not self.vision_received.wait(timeout=2.0):
            print("  Still waiting for /vision/agent_state...")

        objects = self.get_objects()
        print(f"Vision online: {len(objects)} objects detected.")
        for o in objects:
            print(f"   - [{o.get('id', '?')}] {o.get('label', '?')}")

        # Reset vision tracker
        self.vision_reset_pub.publish(String(data='reset'))
        time.sleep(1.0)

        # ── 1. Hold HDD Chassis ─────────────────────────────────────────
        if interactive:
            status, hold_ok = self.run_or_skip(
                "Hold chassis/lid",
                "Find the HDD chassis or lid in vision and secure it with the UF850 gripper.",
                self.step_hold_chassis,
            )
            if status == "quit":
                return
            if status == "skip":
                hold_ok = True
        else:
            hold_ok = self.step_hold_chassis()

        if not hold_ok:
            print("\nABORT: Failed to hold chassis.")
            return

        # ── 2. Unscrew up to 2 screws ──────────────────────────────────
        if interactive:
            status, _ = self.run_or_skip(
                "Unscrew visible screws",
                "Start screw processing. The script will ask again before each individual screw.",
                lambda: self.step_unscrew(max_screws=2, interactive=True),
            )
            if status == "quit":
                return
            if _ == "quit":
                return
        else:
            self.step_unscrew(max_screws=2, interactive=False)

        # ── 3. Flip-drop loose parts ────────────────────────────────────
        if interactive:
            status, flip_drop_ok = self.run_or_skip(
                "Flip-drop loose parts",
                "Flip the held chassis in the drop zone, dump loose parts, and re-grasp the chassis.",
                self.step_flip_drop,
            )
            if status == "quit":
                return
            if status == "skip":
                flip_drop_ok = True
        else:
            flip_drop_ok = self.step_flip_drop()

        if not flip_drop_ok:
            print("\nABORT: Flip-drop failed. Not continuing to backside flip.")
            return

        # ── 4. Flip to expose back side ─────────────────────────────────
        if interactive:
            status, success = self.run_or_skip(
                "Flip chassis to back side",
                "Lift the held chassis, rotate it 180 degrees, descend, release, and re-grasp to expose the back side.",
                lambda: self.flip_skill.execute_flip(interactive=False),
            )
            if status == "quit":
                return
            if status == "skip":
                success = True
        else:
            print("\nExecuting Flip...")
            success = self.flip_skill.execute_flip(interactive=False)

        print("Flip successful." if success else "Flip failed.")

        print("\n" + "#" * 60)
        print("#  HDD DISASSEMBLY SCRIPT — COMPLETE")
        print("#" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = HDDDisassemblyScript()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    executor.add_node(node.hold_skill)
    executor.add_node(node.flip_skill)
    executor.add_node(node.flip_drop_skill)
    executor.add_node(node.unscrew_skill)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(2.0)  # Let subscriptions connect

    try:
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
