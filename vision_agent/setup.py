from setuptools import setup
import os
from glob import glob

package_name = 'vision_agent'

setup(
    name=package_name,
    version='0.0.1',
    # !!! CRITICAL: Include the sub-package here !!!
    packages=[package_name, 'vision_agent.agents'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- NEW ADDITION: Install the launch files ---
        # This tells colcon to copy files from the 'launch' folder to the install directory
        ('share/' + package_name + '/launch', ['launch/start_vision.launch.py', 
                                               'launch/system_startup.launch.py',
                                               'launch/visualize_workspace.launch.py'
                                               ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adip',
    maintainer_email='adip@todo.todo',
    description='PhD Vision System Agents',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This creates the command 'ros2 run vision_agent 
            'start_vision = vision_agent.agent_node:main',
            'start_vision_rtdetr = vision_agent.agent_node_v2:main',
            'detect_workspace = vision_agent.workspace_detector:main',
        ],
    },
)