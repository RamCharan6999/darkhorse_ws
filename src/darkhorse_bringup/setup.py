import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'darkhorse_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all files from your 'launch' and 'config' directories
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darkhorse',
    maintainer_email='darkhorse@todo.todo',
    description='Bringup package for Darkhorse vehicle (launch + configs)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # bringup usually doesn’t have executables, so leave empty
        ],
    },
)

