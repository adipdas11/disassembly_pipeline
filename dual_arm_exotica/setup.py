from glob import glob
import os

from setuptools import setup


package_name = "dual_arm_exotica"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="adip",
    maintainer_email="adipdas11@gmail.com",
    description="EXOTica planning tools for the dual-arm disassembly cell.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dual_arm_ik_planner = dual_arm_exotica.dual_arm_ik_planner:main",
            "dual_arm_trajectory_planner = dual_arm_exotica.dual_arm_trajectory_planner:main",
        ],
    },
)
