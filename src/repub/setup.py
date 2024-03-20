from setuptools import setup
import os

package_name = 'repub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools', 'rclpy', 'pyproj', 'setuptools', 'launch'], #do not include ROS msg packages here
    zip_safe=True,
    maintainer='blackandgold',
    maintainer_email='blackandgold@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'repub_node = repub.repub_node:main',
            'repub_ekf_node = repub.repub_ekf_node:main',
            'joystick_publisher_node = repub.joystick_publisher_node:main',
            'repub_ct_node = repub.repub_ct_node:main',
            'vehicle_inputs_publisher_node = repub.vehicle_inputs_publisher_node:main'
        ],
    },
)
