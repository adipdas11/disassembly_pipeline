#!/usr/bin/env python3

# -----------------------------------------------------------------------------
# Script entrypoint - Python 3.10.6
# -----------------------------------------------------------------------------
import os
import asyncio
import inspect
import re
import ast
import time
import json
import threading
import multiprocessing as mp
import math
from typing import List, Tuple, Optional, Dict, Any

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from langgraph.graph import StateGraph, END

from disassembly_skills.unscrew_skill import UnscrewSkill
from disassembly_skills.object_hold_skill import ObjectHoldSkill
from disassembly_skills.object_flip_skill import ObjectFlipSkill
from disassembly_skills.object_flip_drop_skill import FlipDropSkill
from disassembly_skills.object_pickup_skill import PickupSkill

# -----------------------------------------------------------------------------
# OpenAI (remote) — async client
# -----------------------------------------------------------------------------
from openai import AsyncOpenAI, OpenAIError

DEBUG_FULL_OUTPUT = False
FORBIDDEN_HEADERS = ("Plan:", "Action:", "Observation:", "Final Answer:")
FIRST_LINE_RE = re.compile(r"([^\r\n]*)")
ACTION_RE = re.compile(r"^Action:\s*([\w_]+)\s*\((.*)\)\s*$")

# -----------------------------------------------------------------------------
# 🖥️ NEW: LIVE GUI DASHBOARD PROCESS (WITH COLOR MAPPING)
# -----------------------------------------------------------------------------
def run_live_dashboard(q: mp.Queue):
    """Runs a standalone Tkinter window in a separate process with multi-color highlights."""
    try:
        import tkinter as tk
        from tkinter import scrolledtext
    except ImportError:
        print("Tkinter not installed. Run 'sudo apt-get install python3-tk' for the live window.")
        return

    root = tk.Tk()
    root.title("🧠 LangGraph Autonomous Brain Monitor")
    root.geometry("850x650")
    root.configure(bg="#1e1e1e")

    # Title
    title = tk.Label(root, text="Agent Execution State", fg="white", bg="#1e1e1e", font=("Arial", 16, "bold"))
    title.pack(pady=10)

    # Node Indicators Frame
    node_frame = tk.Frame(root, bg="#1e1e1e")
    node_frame.pack(pady=10)

    # Define stage colors
    node_colors = {
        "VISION": "#FFD700",  # Gold / Yellow
        "THINK": "#9370DB",   # Medium Purple
        "PLAN": "#1E90FF",    # Dodger Blue
        "ACTION": "#FFA500",  # Orange
        "ACT": "#32CD32"      # Lime Green
    }

    nodes = ["VISION", "THINK", "PLAN", "ACTION", "ACT"]
    labels = {}
    
    for n in nodes:
        lbl = tk.Label(node_frame, text=n, width=12, height=2, font=("Arial", 12, "bold"), 
                       bg="#333333", fg="gray", relief="ridge", borderwidth=2)
        lbl.pack(side=tk.LEFT, padx=5)
        labels[n] = lbl

    # Live Log Text Area
    log_area = scrolledtext.ScrolledText(root, wrap=tk.WORD, bg="#000000", font=("Consolas", 11))
    log_area.pack(expand=True, fill='both', padx=20, pady=20)
    
    # Configure text color tags
    for n, hex_color in node_colors.items():
        log_area.tag_config(n, foreground=hex_color)
    
    log_area.insert(tk.END, "Waiting for vision snapshot and mission trigger...\n\n")
    log_area.see(tk.END)

    def update_gui():
        while not q.empty():
            msg = q.get()
            active_node = msg.get("node", "").upper()
            text = msg.get("text", "")

            # Reset all labels to inactive dark gray
            for n in nodes:
                labels[n].config(bg="#333333", fg="gray")

            # Highlight the active node with its specific color
            if active_node in labels:
                bg_color = node_colors.get(active_node, "#00AA00")
                # Use black text on bright colors (like Vision yellow) for contrast, else white
                fg_color = "black" if active_node == "VISION" else "white"
                labels[active_node].config(bg=bg_color, fg=fg_color)

            # Print to log using the corresponding text color tag
            if text:
                log_tag = active_node if active_node in node_colors else None
                log_area.insert(tk.END, f"[{active_node}] {text}\n\n", log_tag)
                log_area.see(tk.END)

        root.after(100, update_gui)

    root.after(100, update_gui)
    root.mainloop()

# -----------------------------------------------------------------------------
# Tool Class + async tools
# -----------------------------------------------------------------------------
class Tool:
    def __init__(self, fn, description: str, args: Optional[Dict[str, str]] = None):
        self.fn = fn
        self.description = description
        self.args = args or {}

# -----------------------------------------------------------------------------
# Terminal color utilities
# -----------------------------------------------------------------------------
class TColor:
    RESET = "\033[0m"
    REASONING   = "\033[38;5;244m"   # soft gray
    PLAN        = "\033[38;5;39m"    # blue
    ACTION      = "\033[38;5;214m"   # orange
    OBSERVATION = "\033[38;5;82m"    # green
    FINAL       = "\033[38;5;201m"   # magenta
    ERROR       = "\033[38;5;196m"   # red
    VISION      = "\033[38;5;226m"   # yellow

def print_stage(text: str):
    if text.startswith("Reasoning:"): color = TColor.REASONING
    elif text.startswith("Plan:"): color = TColor.PLAN
    elif text.startswith("Action:"): color = TColor.ACTION
    elif text.startswith("Observation:"): color = TColor.OBSERVATION
    elif text.startswith("Final Answer:"): color = TColor.FINAL
    else: color = TColor.ERROR
    print(color + text + TColor.RESET, flush=True)

# -----------------------------------------------------------------------------
# Action parser
# -----------------------------------------------------------------------------
def _const_or_name(node: ast.AST) -> Any:
    if isinstance(node, ast.Constant): return node.value
    if isinstance(node, ast.Name): return node.id
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)) and isinstance(node.operand, ast.Constant):
        return -node.operand.value if isinstance(node.op, ast.USub) else +node.operand.value
    raise ValueError("Unsupported argument expression")

def parse_action(response: str) -> Tuple[Optional[str], List[Any], Dict[str, Any]]:
    m = ACTION_RE.match(response.strip())
    if not m: return None, [], {}
    name, args_str = m.group(1), (m.group(2) or "").strip()
    if args_str == "": return name, [], {}

    # Normalize "key: value" syntax to "key=value" for Python ast parsing
    # Handles patterns like: part_id: 0, label: "HDD_Chassis"
    args_str = re.sub(r'(\w+)\s*:\s*', r'\1=', args_str)

    try:
        node = ast.parse(f"f({args_str})", mode="eval")
        call = node.body
        if not isinstance(call, ast.Call): return name, [], {}
        args, kwargs = [], {}
        for a in call.args:
            try: args.append(_const_or_name(a))
            except ValueError: pass
        for kw in call.keywords:
            if kw.arg is None: continue
            try: kwargs[kw.arg] = _const_or_name(kw.value)
            except ValueError: pass
        return name, args, kwargs
    except SyntaxError: return name, [], {}

# -----------------------------------------------------------------------------
# Agent state
# -----------------------------------------------------------------------------
class AgentState(dict):
    input: str
    history: List[str]
    phase: str

# -----------------------------------------------------------------------------
# Master Agent Node
# -----------------------------------------------------------------------------
class MasterAgentNode(Node):
    def __init__(self):
        super().__init__('master_agent')
        self.MISSION_PROMPT = (
            "Your objective is to fully disassemble the assembly in the current scene. "
            "First, analyze the vision data to deduce which object serves as the primary structural base. "
            "PRIORITY RULE: If any screws or word containing 'screw' in the json are visible, you MUST unscrew them all before using any other tools for part removal. "
            "Next, extract all removable sub-components or fasteners visible on the current side one by one. "
            "VERIFICATION RULE: After attempting to remove a macro-component, you must check the next vision frame. "
            "If that specific part is still present, you MUST retry the action. (Note: You do not need to individually verify screws). "
            "Remember that assemblies are 3D objects. Once the current visible side appears completely stripped, "
            "flip the object to expose and analyze the opposite side. "
            "Only output your Final Answer when you have verified that all sides of the primary base are completely empty."
        )

        # 🖥️ START GUI PROCESS
        self.gui_queue = mp.Queue()
        self.gui_process = mp.Process(target=run_live_dashboard, args=(self.gui_queue,))
        self.gui_process.daemon = True
        self.gui_process.start()

        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.loop_thread.start()
        
        self.vision_lock = threading.Lock()
        self.detected_objects = []
        self.has_printed_startup_vision = False
        self.auto_start_triggered = False

        # --- NEW: SPATIAL MEMORY COMPLETION LOG ---
        self.cleared_zones = []       # Stores XYZ tuples of removed parts
        self.EXCLUSION_RADIUS = 0.010 # 15mm spherical blind spot tolerance

        API_KEY_FILE = "/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/config/api_key.txt"
        try:
            with open(API_KEY_FILE, "r") as f: OPENAI_API_KEY = f.read().strip()
        except FileNotFoundError: raise RuntimeError(f"Missing {API_KEY_FILE}.")

        self.OPENAI_MODEL = os.getenv("OPENAI_MODEL", "gpt-5.1")
        self.oai_client = AsyncOpenAI(api_key=OPENAI_API_KEY)

        self.unscrew_skill = UnscrewSkill()
        self.hold_skill = ObjectHoldSkill()
        self.flip_skill = ObjectFlipSkill()
        self.flip_drop_skill = FlipDropSkill()
        self.pickup_skill = PickupSkill()

        self.tools: Dict[str, Tool] = {
            "hold_object": Tool(
                self.hold_object,
                "Secures the main device chassis to the table using tactile feedback. Must be done before unscrewing or part extraction. Requires the ID and label of the part to be held.",
                args={"part_id": "int: ID of the holding target", "label": "str: label of the holding target"}
            ),
            "unscrew": Tool(
                self.unscrew,
                "Unthreads a screw. Requires the ID and label of the target screw.",
                args={"unscrew_id": "int: ID of the target screw", "unscrew_label": "str: label of the target screw"}
            ),
            "flip_object": Tool(
                self.flip_object,
                "Flips the main chassis over to expose the back side. Use this when you need to access parts or screws located on the underside of the device."
            ),
            "pickup_object": Tool(
                self.pickup_object,
                "Uses the precision gripper to safely extract delicate internal components. "
                "MANDATORY RULE: Use this tool ONLY for components labeled as 'PCB' in the vision data. "
                "For any other component (lids, frames, modules) use 'flip_drop' for removal instead. "
                "CRITICAL WARNING: Using this tool causes the robot to release the main chassis. "
                "You MUST call the 'hold_object' tool immediately after this action succeeds to re-secure the workspace. "
                "Requires the ID and label of the target object.",
                args={"pickup_id": "int: ID of the target object", "pickup_label": "str: label of the target object"}
            ),
            "flip_drop": Tool(
                self.flip_drop,
                "The primary clearing action for all non-PCB components. It flips the entire chassis upside down to dump out loose unthreaded screws, detached lids, and other non-delicate internal modules. "
                "STRATEGY: Use this to remove everything EXCEPT PCBs once all relevant screws are unthreaded. "
                "WARNING: Ensure all 'PCB' items have been picked up using 'pickup_object' before using this tool."
            ),
        }

        self.app = self._build_graph()
        self.goal_sub = self.create_subscription(String, '/agent_goal', self.goal_callback, 10)
        self.vision_sub = self.create_subscription(String, '/vision/agent_state', self.vision_callback, 10)
        
        self.get_logger().info("🤖 Master Agent Ready. Awaiting Vision for Auto-Disassembly.")

    def _run_async_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    # -----------------------------------------------------------------------------
    # Tools implementations
    # -----------------------------------------------------------------------------
    async def hold_object(self, part_id=None, label=None):
        if part_id is None or label is None: return "Action failed, missing required parameter"
        self.hold_skill.execute_hold(part_id=part_id, target_label=label, interactive=False)
        return "Tool called successfully"

    async def unscrew(self, unscrew_id=None, unscrew_label=None):
        if unscrew_id is None or unscrew_label is None: return "Action failed, missing required parameter"
        
        # --- NEW: Retrieve XYZ before action to log to Spatial Memory ---
        target_xyz = None
        with self.vision_lock:
            for obj in self.detected_objects:
                if obj.get('id') == unscrew_id:
                    target_xyz = obj.get('xyz')
                    break

        self.unscrew_skill.execute_unscrew_command(target_id=unscrew_id, target_label=unscrew_label, interactive=False)
        
        # --- NEW: Add coordinates to Exclusion Zones ---
        if target_xyz:
            self.cleared_zones.append(target_xyz)
            print(f"🛑 [MEMORY] Added Exclusion Zone at {target_xyz} (Radius: {self.EXCLUSION_RADIUS*1000}mm)")
            
        return "Tool called successfully"

    # 🆕 NEW ASYNC WRAPPERS FOR NEW SKILLS
    async def flip_object(self):
        self.flip_skill.execute_flip(interactive=False)
        return "Tool called successfully: Object flipped"

    async def flip_drop(self):
        self.flip_drop_skill.execute_flip_drop(interactive=False)
        return "Tool called successfully: Object dropped"

    async def pickup_object(self, pickup_id=None, pickup_label=None):
        if pickup_id == None or pickup_label == None:
            return "Action failed, missing pickup_id or pickup_label parameter"
        self.pickup_skill.execute_pickup(target_id=pickup_id, target_label=pickup_label, interactive=False)
        return "Tool called successfully"

    # -----------------------------------------------------------------------------
    # Vision handling
    # -----------------------------------------------------------------------------
    def vision_callback(self, msg):
        try:
            data = json.loads(msg.data.strip("'"))
            raw_objects = data.get("global_view", {}).get("objects", [])
            
            # --- NEW: Spatial Memory Filtering ---
            temp_list = []
            for o in raw_objects:
                obj_id = o.get("id")
                obj_label = o.get("label")
                obj_xyz = o.get("xyz")
                
                # If no XYZ data exists, pass it through safely
                if not obj_xyz or len(obj_xyz) < 3:
                    temp_list.append({"id": obj_id, "label": obj_label, "xyz": obj_xyz})
                    continue
                    
                is_cleared = False
                for cleared_xyz in self.cleared_zones:
                    # Calculate 3D Euclidean distance
                    dist = math.hypot(
                        obj_xyz[0] - cleared_xyz[0],
                        obj_xyz[1] - cleared_xyz[1],
                        obj_xyz[2] - cleared_xyz[2]
                    )
                    # If it falls inside the blind spot, flag it for deletion
                    if dist < self.EXCLUSION_RADIUS:
                        is_cleared = True
                        break
                        
                # Only add objects to the LLM's view if they are not inside a cleared zone
                if not is_cleared:
                    temp_list.append({"id": obj_id, "label": obj_label, "xyz": obj_xyz})
            # -------------------------------------

            with self.vision_lock:
                self.detected_objects = temp_list
                
                if not self.has_printed_startup_vision and len(self.detected_objects) > 0:
                    vis_text = f"Startup Snapshot ({len(self.detected_objects)} valid objects detected)."
                    print(f"\n{TColor.VISION}Vision: {vis_text}{TColor.RESET}")
                    
                    self.gui_queue.put({"node": "VISION", "text": vis_text})
                    self.has_printed_startup_vision = True

                if not self.auto_start_triggered and len(self.detected_objects) > 0:
                    self.auto_start_triggered = True
                    initial_state = {"input": self.MISSION_PROMPT, "history": [], "phase": "plan"}
                    asyncio.run_coroutine_threadsafe(self.run_agent(initial_state), self.loop)
        except Exception: pass

    # -----------------------------------------------------------------------------
    # Prompt builders
    # -----------------------------------------------------------------------------
    def format_tool_list(self) -> str:
        lines = []
        for name, tool in self.tools.items():
            if tool.args:
                args_str = ", ".join(f"{arg}: {desc}" for arg, desc in tool.args.items())
                lines.append(f"- {name}({args_str}): {tool.description}")
            else:
                lines.append(f"- {name}(): {tool.description}")
        return "\n".join(lines)

    def build_think_prompt(self, user_input: str, history: List[str]) -> str:
        tool_list = self.format_tool_list()
        with self.vision_lock: vision_str = json.dumps(self.detected_objects)
        guide = "\n".join([
            "You are a part of a ReAct agent. Your role is to produce a reasoning stage to reason about the problem and guide the following Plan, Action and Observation stages that will be handled by the rest of the agent.",
            "This stage is a TRANSPARENT scratchpad that will be shown to the user.",
            "",
            "Rules:",
            "- Output MUST start with: Reasoning:",
            "- You may write multiple lines after 'Reasoning:'.",
            "- Do NOT output Plan:, Action:, Observation:, or Final Answer: in this stage.",
            "- Do NOT refer to your role in the output. only produce relevant reasoning that can be used by the other parts of the agent",
            "- Be concrete: if applicable, summarise the previous plan> action> observation within the context of the user query and create a short generation to support the next planning stage",
            "",
            "Current Scene:",
            vision_str])
        return "\n".join([guide, "Tools available:", tool_list, "", f"User: {user_input}", *history, ""])

    def build_plan_prompt(self, user_input: str, history: List[str]) -> str:
        tool_list = self.format_tool_list()
        guide = "\n".join([
            "You are a ReAct agent controlling a robot arm.",
            "Output exactly ONE line.",
            "It MUST start with: Plan:  (or you may output Final Answer: if the task is complete).",
            "",
            "Rules:",
            "- Output exactly ONE line only.",
            "- Do NOT output Action:, Observation:, or Reasoning: here.",
            "- If uncertain, make a cautious Plan that leads to an Action next.",
            ""])
        return "\n".join([guide, "Tools available:", tool_list, "", f"User: {user_input}", *history, ""])

    def build_action_prompt(self, user_input: str, history: List[str]) -> str:
        tool_list = self.format_tool_list()
        guide = "\n".join([
            "You are a ReAct agent controlling a robot arm.",
            "Output exactly ONE line.",
            "It MUST start with: Action:",
            "",
            "Rules:",
            "- Output exactly ONE line only.",
            "- Do NOT output Plan:, Observation:, Reasoning:, or Final Answer: here.",
            "- Use ONLY the tool names/signatures exactly as listed.",
            "- Do not invent extra keyword arguments.",
            ""])
        return "\n".join([guide, "Tools available:", tool_list, "", f"User: {user_input}", *history, ""])

    # -----------------------------------------------------------------------------
    # Remote LLM calls
    # -----------------------------------------------------------------------------
    async def call_llm_remote_first_line(self, prompt: str, max_tokens: int = 200, timeout_s: int = 180) -> str:
        try:
            resp = await asyncio.wait_for(
                self.oai_client.responses.create(
                    model=self.OPENAI_MODEL, reasoning={"effort": "none"}, input=prompt, max_output_tokens=max_tokens
                ), timeout=timeout_s)
            text = getattr(resp, "output_text", None)
            if not text:
                parts = []
                for item in resp.output or []:
                    if getattr(item, "type", None) == "message":
                        for c in (item.content or []):
                            if getattr(c, "type", None) == "output_text": parts.append(getattr(c, "text", ""))
                text = "".join(parts).strip()
            if not text: return ""
            m = FIRST_LINE_RE.match(text.strip())
            return m.group(1).strip() if m else text.splitlines()[0].strip()
        except (asyncio.TimeoutError, OpenAIError) as e:
            return f"Plan: (API error: {type(e).__name__}) proceed cautiously."

    async def call_llm_remote_full(self, prompt: str, max_tokens: int = 800, timeout_s: int = 180) -> str:
        try:
            resp = await asyncio.wait_for(
                self.oai_client.responses.create(
                    model=self.OPENAI_MODEL, reasoning={"effort": "none"}, input=prompt, max_output_tokens=max_tokens
                ), timeout=timeout_s)
            text = getattr(resp, "output_text", None)
            if not text:
                parts = []
                for item in resp.output or []:
                    if getattr(item, "type", None) == "message":
                        for c in (item.content or []):
                            if getattr(c, "type", None) == "output_text": parts.append(getattr(c, "text", ""))
                text = "".join(parts).strip()
            return text.strip() if text else ""
        except (asyncio.TimeoutError, OpenAIError) as e:
            return f"Reasoning: (API error: {type(e).__name__})"

    # -----------------------------------------------------------------------------
    # Reasoning sanitizer
    # -----------------------------------------------------------------------------
    def sanitize_reasoning(self, txt: str) -> str:
        txt = (txt or "").strip()
        if not txt.startswith("Reasoning:"): txt = "Reasoning:\n" + txt
        lines = txt.splitlines()
        out = [lines[0]]
        for line in lines[1:]:
            if any(line.strip().startswith(h) for h in FORBIDDEN_HEADERS): continue
            out.append(line)
        return "\n".join(out).strip()

    # -----------------------------------------------------------------------------
    # LangGraph nodes
    # -----------------------------------------------------------------------------
    async def think(self, state: AgentState) -> AgentState:
        txt = await self.call_llm_remote_full(self.build_think_prompt(state["input"], state["history"]), max_tokens=800)
        txt = self.sanitize_reasoning(txt)
        state["history"].append(txt)
        print_stage(txt)
        return state

    async def plan_node(self, state: AgentState) -> AgentState:
        line = await self.call_llm_remote_first_line(self.build_plan_prompt(state["input"], state["history"]), max_tokens=120)
        if line.startswith("Final Answer:"):
            state["history"].append(line); print_stage(line); return state
        if not line.startswith("Plan:"): line = "Plan: Prepare next simple step"
        state["history"].append(line); print_stage(line); state["phase"] = "action"
        return state

    async def action_node(self, state: AgentState) -> AgentState:
        if state["history"] and state["history"][-1].startswith("Final Answer:"): return state
        line = await self.call_llm_remote_first_line(self.build_action_prompt(state["input"], state["history"]), max_tokens=120)
        if not line.startswith("Action:"):
            # LLM failed to produce a valid action — skip this cycle and re-think
            state["history"].append("Observation: No valid action produced. Re-evaluating.")
            print_stage("Observation: No valid action produced. Re-evaluating.")
            return state
        # Validate tool name before accepting
        name, _, _ = parse_action(line)
        if name and name not in self.tools:
            state["history"].append(f"Observation: Unknown tool '{name}'. Available: {', '.join(self.tools.keys())}")
            print_stage(f"Observation: Unknown tool '{name}'. Use one of: {', '.join(self.tools.keys())}")
            return state
        state["history"].append(line); print_stage(line)
        return state

    async def act(self, state: AgentState) -> AgentState:
        if not state["history"]: return state
        last = state["history"][-1]
        if last.startswith("Final Answer:"): return state
        if last.startswith("Action:"):
            name, args, kwargs = parse_action(last)
            tool = self.tools.get(name or "")
            if not tool: obs = f"Observation: Unknown tool '{name}'."
            else:
                try:
                    if inspect.iscoroutinefunction(tool.fn): result = await tool.fn(*args, **kwargs)
                    else: result = tool.fn(*args, **kwargs)
                    obs = f"Observation: {result}"
                except Exception as e: obs = f"Observation: Tool errored: {e}"
            state["history"].append(obs); print_stage(obs)
            self.gui_queue.put({"node": "ACT", "text": obs})
        return state

    # -----------------------------------------------------------------------------
    # Runner & Graph Compile
    # -----------------------------------------------------------------------------
    def _build_graph(self):
        graph = StateGraph(AgentState)
        graph.add_node("think", self.think)
        graph.add_node("plan", self.plan_node)
        graph.add_node("action", self.action_node)
        graph.add_node("act", self.act)
        graph.set_entry_point("think")
        graph.add_edge("think", "plan")
        graph.add_edge("plan", "action")
        graph.add_edge("action", "act")
        def _branch(s: AgentState) -> str:
            return "end" if any(line.startswith("Final Answer:") for line in s["history"]) else "continue"
        graph.add_conditional_edges("act", _branch, {"continue": "think", "end": END})
        
        app = graph.compile()
        try:
            with open("agent_architecture.md", "w") as f:
                f.write("```mermaid\n")
                f.write(app.get_graph().draw_mermaid())
                f.write("\n```")
        except Exception: pass
        return app

    async def run_agent(self, state: AgentState):
        print("PROMPT:", state["input"], flush=True)
        self.gui_queue.put({"node": "VISION", "text": f"Mission Started: {state['input']}"})
        
        async for event in self.app.astream(state, config={"recursion_limit": 400}):
            current_node = list(event.keys())[0]
            latest_history = state["history"][-1] if state["history"] else ""
            self.gui_queue.put({"node": current_node, "text": latest_history})
            
        return state["history"]

    def goal_callback(self, msg):
        initial_state = {"input": msg.data, "history": [], "phase": "plan"}
        asyncio.run_coroutine_threadsafe(self.run_agent(initial_state), self.loop)

# -----------------------------------------------------------------------------
# Script entrypoint
# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MasterAgentNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)
    executor.add_node(node.unscrew_skill)
    executor.add_node(node.hold_skill)
    executor.add_node(node.flip_skill)
    executor.add_node(node.flip_drop_skill)
    executor.add_node(node.pickup_skill)
    
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()