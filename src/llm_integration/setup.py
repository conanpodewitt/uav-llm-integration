from setuptools import setup
from glob import glob
import os

package_name = 'llm_integration'

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
    description='Integration of LLM API for motion control',
    license='MIT License',
    entry_points={
        'console_scripts': [
            'llm_node = llm_integration.llm_node:main',
            'ui_node = llm_integration.ui_node:main',
            'camera_caption_node = llm_integration.camera_caption_node:main',
        ],
    },
)
