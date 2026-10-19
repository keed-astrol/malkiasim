from setuptools import setup
import os
from glob import glob

package_name = 'dojo_bt'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'),
         glob('behavior_trees/**/*.xml', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keedastro',
    maintainer_email='keedastro@users.noreply.github.com',
    description='Custom Behavior Tree nodes and autonomous mission manager for RoboDojo',
    license='BSD',
    entry_points={
        'console_scripts': [
            'autonomous_mission_node = dojo_bt.autonomous_mission_node:main',
            'level_scan_gate = dojo_bt.level_scan_gate:main',
        ],
    },
)
