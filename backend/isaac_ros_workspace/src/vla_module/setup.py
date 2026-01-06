from setuptools import setup
import os
from glob import glob

package_name = 'vla_module'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pyY]')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.[pyY]')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@todo.todo',
    description='Vision-Language-Action (VLA) module for voice-command-based humanoid robot control',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_capture = voice_recognition.audio_capture:main',
            'whisper_interface = voice_recognition.whisper_interface:main',
            'voice_processor = voice_recognition.voice_processor:main',
            'voice_command_publisher = voice_recognition.voice_command_publisher:main',
            'command_parser = cognitive_planning.command_parser:main',
            'task_planner = cognitive_planning.task_planner:main',
            'action_converter = cognitive_planning.action_converter:main',
            'task_execution_manager = task_execution.task_execution_manager:main',
        ],
    },
)