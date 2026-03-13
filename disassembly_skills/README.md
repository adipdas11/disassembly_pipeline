# Disassembly Skills

## Motion Backend API

The `MotionBackend` class provides an interface for controlling robot motion using MoveIt 2. It supports joint-space movement and robust Cartesian path planning with fallback strategies.

### `MotionBackend`

**Initialization:**
```python
MotionBackend(node: Node, group_name: str)
```
- `node` (rclpy.node.Node): The ROS 2 node instance.
- `group_name` (str): The name of the MoveIt planning group (e.g., "xarm6", "ur5_manipulator").

**Methods:**

#### `move_to_joint_positions`
Moves the robot to a set of target joint positions.
```python
move_to_joint_positions(self, target_joints: dict, filter_prefix: str = "") -> bool
```
- `target_joints` (dict): A dictionary mapping joint names to target positions (in radians).
- `filter_prefix` (str, optional): A prefix to filter joint names from the current state. Useful when `target_joints` only contains a subset of joints. Defaults to `""`.
- **Returns**: `True` if the movement was successful, `False` otherwise.

#### `move_to_pose_robust`
Computes an Inverse Kinematics (IK) solution for a target pose and moves the robot. It includes a robust fallback mechanism that relaxes orientation constraints if a strict solution is not found.
```python
move_to_pose_robust(self, x, y, z, q_dict, link_name, frame_id='world_world') -> bool
```
- `x`, `y`, `z` (float): The target position coordinates.
- `q_dict` (dict): A dictionary containing quaternion orientation `{'qx', 'qy', 'qz', 'qw'}`. If provided and valid, the solver attempts to reach this specific orientation first.
- `link_name` (str): The name of the link to position (e.g., end-effector link).
- `frame_id` (str, optional): The reference frame for the target pose. Defaults to `'world_world'`.
- **Returns**: `True` if a valid IK solution was found and executed, `False` otherwise.

**Robust IK Strategy:**
1.  **Strict Orientation**: If `q_dict` is provided, attempts to find a solution matching the exact position and orientation.
2.  **Vertical Relaxation**: If strict fails, attempts a vertical orientation (pointing down, roll=pi, pitch=0, yaw=0).
3.  **Yaw Relaxation**: If vertical fails, searches a window of yaw angles (rotations around Z) while maintaining vertical pitch/roll to find a reachable pose.
