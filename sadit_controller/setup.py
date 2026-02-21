from setuptools import setup
import os
from glob import glob

package_name = 'sadit_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyalarını ekle
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        # Config dosyalarını ekle
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        #Model dosyası için
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='furkan', # Kendi adınla değiştirebilirsin
    maintainer_email='furkan@example.com', # Kendi e-postanla değiştirebilirsin
    description='Controller package for Sadit robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower_node = sadit_controller.line_follower_node:main',
            'yolov8_line_follower = sadit_controller.yolov8_line_follower_node:main',
            'opencv_line_follower = sadit_controller.opencv_line_follower_node:main'
        ],
    },
)
