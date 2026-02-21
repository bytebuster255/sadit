from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sadit_serial_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='gunes',
    maintainer_email='hikmetselcukgunes@gmail.com',
    description='ROS2 Arduino Serial Control Package for SADIT Robot Movement',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sadit_serial_controller = sadit_serial_control.sadit_serial_controller:main',
        ],
    },
)
