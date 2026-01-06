from setuptools import setup
from glob import glob
import os

package_name = 'vla_msgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all message files
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        # Include all action files
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
        # Include all service files
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@todo.todo',
    description='Messages for Vision-Language-Action (VLA) module',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)