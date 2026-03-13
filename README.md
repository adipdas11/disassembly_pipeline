# Disassembly Pipeline

ROS 2 workspace source tree for a dual-arm disassembly cell. This repository contains the MoveIt configuration and launch flow for the dual-arm system, robot and scene descriptions, tool and camera nodes, skill code, and a separate `vision_training/` workspace for dataset preparation and model training.

<details>
<summary><strong>1. Clone Plan</strong></summary>

Clone into a ROS 2 workspace source directory:

```bash
mkdir -p ~/workspace/dev_ws/src
cd ~/workspace/dev_ws/src
git clone https://github.com/adipdas11/src.git disassembly_pipeline
cd ~/workspace/dev_ws
```

Expected layout after clone:

```text
~/workspace/dev_ws/
├── src/
│   └── disassembly_pipeline/
└── ...
```

</details>

<details>
<summary><strong>2. ROS 2 Dependencies</strong></summary>

This repo is organized as a ROS 2 workspace source tree and is intended to be built from the workspace root.

Recommended system setup:

- ROS 2 Humble installed and sourced
- `colcon` and `rosdep` installed
- Python 3.10 available

Install ROS package dependencies from the workspace root:

```bash
cd ~/workspace/dev_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
cd ~/workspace/dev_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Targeted rebuild:

```bash
colcon build --symlink-install --packages-select dual_arm_moveit_config tool_controller tool_camera_pkg vision_agent disassembly_skills
```

</details>

<details>
<summary><strong>3. Vision Training Environment</strong></summary>

The `vision_training/` directory has its own Python environment and dependency lockfiles:

- [vision_training/pyproject.toml](/home/adip/workspace/dev_ws/src/disassembly_pipeline/vision_training/pyproject.toml)
- `vision_training/uv.lock`

Recommended setup with `uv`:

```bash
cd ~/workspace/dev_ws/src/disassembly_pipeline/vision_training
uv sync
source .venv/bin/activate
```

Run tools inside the managed environment:

```bash
uv run python your_script.py
uv run jupyter lab
```

If you do not want to use `uv`, a plain virtual environment also works, but you will need to install dependencies manually:

```bash
cd ~/workspace/dev_ws/src/disassembly_pipeline/vision_training
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e .
```

The vision environment currently depends on packages such as `rfdetr`, `ultralytics`, `opencv-python`, `roboflow`, `transformers`, `torch`, and `torchvision`.

</details>

<details>
<summary><strong>4. How To Launch The Demo</strong></summary>

Build and source first:

```bash
cd ~/workspace/dev_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Main demo launch:

```bash
ros2 launch dual_arm_moveit_config demo.launch.py
```

Launch with an explicit hardware type:

```bash
ros2 launch dual_arm_moveit_config demo.launch.py hardware_type:=real
ros2 launch dual_arm_moveit_config demo.launch.py hardware_type:=fake
ros2 launch dual_arm_moveit_config demo.launch.py hardware_type:=isaac
```

Servo can also be toggled explicitly:

```bash
ros2 launch dual_arm_moveit_config demo.launch.py hardware_type:=real enable_servo:=true
```

</details>

<details>
<summary><strong>5. Hardware Types</strong></summary>

- `real`
  Uses the Python hardware bridge, real robot state publishing, tool control, and the real MoveIt / Servo stack.

- `fake`
  Runs the MoveIt and controller stack without real hardware, useful for development and dry runs.

- `isaac`
  Intended for an external simulator-managed controller setup.

</details>

<details>
<summary><strong>6. Main Packages</strong></summary>

- `dual_arm_moveit_config/`
  MoveIt config, launch files, RViz config, hardware bridge, teleop bridge, and scripts.

- `tool_controller/`
  Tool command node and launch/config for tool actuation.

- `tool_camera_pkg/`
  Tool camera ROS 2 nodes and configs.

- `scene_description/` and `robots/`
  Scene URDF/Xacro, robot descriptions, and meshes.

- `disassembly_skills/`
  Higher-level runtime skill logic and supporting configs.

- `vision_agent/`
  Runtime vision-related ROS 2 package.

- `vision_training/`
  Training utilities, datasets, notebooks, and model workspaces.

</details>
