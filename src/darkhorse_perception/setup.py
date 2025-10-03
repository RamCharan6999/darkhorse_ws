from setuptools import setup
from glob import glob
import os

package_name = 'darkhorse_perception'

def files_in(rel_path, patterns):
    files = []
    for pat in patterns:
        files += glob(os.path.join(rel_path, pat))
    return files

launch_files  = files_in('launch',  ['*.py'])
config_files  = files_in('config',  ['*.yaml', '*.yml'])
rviz_files    = files_in('rviz',    ['*.rviz'])
model_files   = files_in('models',  ['*.onnx', '*.pt', '*.txt'])

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', launch_files),
    ('share/' + package_name + '/config', config_files),
    ('share/' + package_name + '/rviz',   rviz_files),
    ('share/' + package_name + '/models', model_files),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darkhorse',
    maintainer_email='',
    description='Perception nodes',
    license='',
    entry_points={
        'console_scripts': [
            'camera_node = darkhorse_perception.camera_node:main',
            'radar_rx_node = darkhorse_perception.radar_rx_node:main',
            'radar_visualizer = darkhorse_perception.radar_visualizer:main',
            'vehicle_speed_node = darkhorse_perception.vehicle_speed_node:main',

        ],
    },
)

