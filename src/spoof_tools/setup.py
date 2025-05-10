from setuptools import setup
from glob import glob
import os

package_name = 'spoof_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install marker file for ament index
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [os.path.join('resource', package_name)]),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install package.xml
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Conan Po Dewitt',
    maintainer_email='22877792@student.uwa.edu.au',
    description='Prompt poisoning and sensor spoofing tools',
    license='MIT License',
    entry_points={
        'console_scripts': [
            'poison_node = spoof_tools.poison_node:main',
            'spoof_node = spoof_tools.spoof_node:main',
        ],
    },
)