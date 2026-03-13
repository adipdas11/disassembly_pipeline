"""Joint naming and layout helpers for the dual-arm EXOTica planners."""

JOINT_NAMES = [
    "xarm5_joint1",
    "xarm5_joint2",
    "xarm5_joint3",
    "xarm5_joint4",
    "xarm5_joint5",
    "u1_joint1",
    "u1_joint2",
    "u1_joint3",
    "u1_joint4",
    "u1_joint5",
    "u1_joint6",
    "rg6_l_out",
    "slider_slider_joint",
]

REVOLUTE_JOINTS = {
    "xarm5_joint1",
    "xarm5_joint2",
    "xarm5_joint3",
    "xarm5_joint4",
    "xarm5_joint5",
    "u1_joint1",
    "u1_joint2",
    "u1_joint3",
    "u1_joint4",
    "u1_joint5",
    "u1_joint6",
}

XARM_JOINTS = JOINT_NAMES[0:5]
UF_JOINTS = JOINT_NAMES[5:11]
RG6_JOINT = JOINT_NAMES[11]
SLIDER_JOINT = JOINT_NAMES[12]
UF_PLANNING_JOINTS = UF_JOINTS
