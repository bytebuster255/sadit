from setuptools import setup
import os
from glob import glob

package_name = 'sdt_perception'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools','numpy','opencv-python'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Lane detection with offset and debug image',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_follower = sdt_perception.lane_follower:main',
        ],
    },
)
