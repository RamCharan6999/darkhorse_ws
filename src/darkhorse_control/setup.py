from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'darkhorse_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 👇 add this line to install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darkhorse',
    maintainer_email='darkhorse@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'acc_node = darkhorse_control.acc_node:main',
            'aeb_node = darkhorse_control.aeb_node:main',
            'joy_bridge = darkhorse_control.joy_bridge:main',
            'mission_control_node = darkhorse_control.mission_control_node:main',
            'fake_speed_publisher = darkhorse_control.fake_speed_publisher:main',
            'aeb_controller = darkhorse_control.aeb_controller:main',
            'lka_node = darkhorse_control.lka_node:main',
        ],
    },
)

