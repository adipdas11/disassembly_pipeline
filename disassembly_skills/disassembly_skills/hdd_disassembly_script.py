#!/usr/bin/env python3
"""
HDD Disassembly Script — Pure state-machine, no LLM.

Flow:
  1. Subscribe to vision, wait for detections
  2. Hold the HDD chassis
  3. Pickup PCB main
  4. Re-hold chassis → flip
  5. Done
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
import json, time, threading, math, copy

from disassembly_skills.object_hold_skill import ObjectHoldSkill
from disassembly_skills.object_flip_skill import ObjectFlipSkill
from disassembly_skills.object_pickup_skill import PickupSkill


class HDDDisassemblyScript(Node):
    def __init__(self):
        super().__init__('hdd_disassembly_script')

        # ── Skill nodes (added to executor separately) ──────────────────
        self.hold_skill    = ObjectHoldSkill()
        self.flip_skill    = ObjectFlipSkill()
        self.pickup_skill  = PickupSkill()

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

        self.get_logger().info("🚀 HDD Disassembly Script: Ready.")

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

    def find_pcb(self, objects):
        return self.find_by_keywords(objects, ["pcb"])

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
                print(f"  ⚠️ No HDD chassis/lid detected (attempt {attempt+1}/3). Waiting...")
                time.sleep(2.0)
                continue

            target = chassis_list[0]
            tid, tlabel = target.get("id"), target.get("label", "unknown")
            print(f"  🎯 Found: {tlabel} (ID: {tid})")
            print(f"  🗜️ Calling hold_skill.execute_hold({tid}, '{tlabel}')...")

            success = self.hold_skill.execute_hold(
                part_id=tid, target_label=tlabel, interactive=False
            )
            if success:
                print("  ✅ Hold successful.")
                return True
            else:
                print("  ❌ Hold failed. Retrying...")
                time.sleep(1.0)

        print("  ❌ Could not hold chassis after 3 attempts.")
        return False

    def step_pickup_pcb(self):
        """Pick up the PCB main and verify it's no longer visible."""
        print("\n" + "=" * 60)
        print("STEP: PICKUP PCB MAIN")
        print("=" * 60)

        MAX_RETRIES = 3
        for attempt in range(MAX_RETRIES):
            self.wait_for_vision()
            objects = self.get_objects()
            pcbs = self.find_pcb(objects)

            if not pcbs:
                print("  ✅ No PCB detected — already removed or not present.")
                return True

            target = pcbs[0]
            tid, tlabel = target.get("id"), target.get("label", "unknown")
            print(f"  📦 Picking up: {tlabel} (ID: {tid}) — attempt {attempt+1}/{MAX_RETRIES}")

            self.pickup_skill.execute_pickup(
                target_id=tid, target_label=tlabel
            )

            # ── Verify removal in vision ────────────────────────────────
            print("  🔍 Verifying removal in vision...")
            time.sleep(2.0)
            self.wait_for_vision()
            objects_after = self.get_objects()
            pcbs_after = self.find_pcb(objects_after)

            if not pcbs_after:
                print("  ✅ PCB removed successfully — not visible in vision.")
                return True
            else:
                print(
                    f"  ⚠️ PCB still detected ({len(pcbs_after)} found). "
                    f"Retrying pickup..."
                )
                time.sleep(1.0)

        print(f"  ❌ Could not remove PCB after {MAX_RETRIES} attempts.")
        return False

    def run(self):
        print("\n" + "#" * 60)
        print("#  HDD DISASSEMBLY SCRIPT — START")
        print("#" * 60)

        # ── Wait for first vision frame ─────────────────────────────────
        print("\n⏳ Waiting for vision data...")
        while not self.vision_received.wait(timeout=2.0):
            print("  Still waiting for /vision/agent_state...")

        objects = self.get_objects()
        print(f"👁️ Vision online: {len(objects)} objects detected.")
        for o in objects:
            print(f"   - [{o.get('id', '?')}] {o.get('label', '?')}")

        # Reset vision tracker
        self.vision_reset_pub.publish(String(data='reset'))
        time.sleep(1.0)

        # ── 1. Hold HDD Chassis ─────────────────────────────────────────
        if not self.step_hold_chassis():
            print("\n❌ ABORT: Failed to hold chassis.")
            return

        # ── 2. Pickup PCB Main ──────────────────────────────────────────
        if not self.step_pickup_pcb():
            print("\n⚠️ PCB pickup failed, but continuing...")

        # ── 3. Hold HDD Chassis Again (as pickup releases gripper) ──────
        print("\n🔄 Re-stabilizing chassis for flip...")
        if not self.step_hold_chassis():
            print("\n❌ ABORT: Failed to re-hold chassis.")
            return

        # ── 4. Flip ─────────────────────────────────────────────────────
        print("\n🔄 Executing Flip...")
        success = self.flip_skill.execute_flip(interactive=False)
        if success:
            print("✅ Flip successful.")
        else:
            print("❌ Flip failed.")

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
    executor.add_node(node.pickup_skill)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(2.0)  # Let subscriptions connect

    try:
        node.run()
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
