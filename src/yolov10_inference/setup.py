from setuptools import find_packages, setup
import os
from glob import glob  

package_name = 'yolov10_inference'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'supervision',
        'ultralytics'
    ],
    zip_safe=True,
    maintainer='tony',
    maintainer_email='3369883232@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'yolo_rgs = yolov10_inference.yolo_rgs:main',
        ],
    },
)
