from setuptools import setup

package_name = 'ax12a_arm_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ax12a_arm.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='5-DOF AX-12A arm driver using DYNAMIXEL SDK (ROS 2 Humble)',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ax12a_arm_node = ax12a_arm_controller.ax12a_arm_node:main',
        ],
    },
)
