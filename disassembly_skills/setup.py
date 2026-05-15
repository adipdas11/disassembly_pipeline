from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'disassembly_skills'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add these two lines:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adip',
    maintainer_email='adipdas11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [ 
            'motion_backend = disassembly_skills.motion_backend:main',
            'object_hold_skill = disassembly_skills.object_hold_skill:main',
            'object_flip_skill = disassembly_skills.object_flip_skill:main',
            'object_flip_drop_skill = disassembly_skills.object_flip_drop_skill:main',
            'object_pickup_skill = disassembly_skills.object_pickup_skill:main',
            'hdd_disassembly_script = disassembly_skills.hdd_disassembly_script:main',
            'hdd_react_ui_script = disassembly_skills.hdd_react_ui_script:main',
            'unscrew_skill = disassembly_skills.unscrew_skill:main',
            'master_agent = disassembly_skills.master_agent:main',
            'groq_master_agent = disassembly_skills.groq_master_agent:main',
        ],
    },
)
