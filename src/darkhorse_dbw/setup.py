import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'darkhorse_dbw'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs all files from your 'config' directory
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darkhorse',
    maintainer_email='darkhorse@todo.todo',
    description='Package for Darkhorse Drive-by-Wire and HAL',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hal_node = darkhorse_dbw.hal_node:main',
            'speed_node = darkhorse_dbw.nodes.speed_node:main',

        ],
    },
)

