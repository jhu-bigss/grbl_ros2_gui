import os
from glob import glob
from setuptools import setup, find_packages
# from setuptools import find_packages

package_name = 'grbl_ros2_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include model and simulation files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*'))
    ],
    install_requires=['setuptools',
                      'PyQt5',
                      'pyserial',
                      'LabJackPython',
                      ],
    zip_safe=True,
    maintainer='Joshua Liu',
    maintainer_email='liushuya7@gmail.com',
    description='A ROS2 GUI for interfacing with grbl devices',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grbl_gui = grbl_ros2_gui.grbl_gui:main',
            'labjack_stream_device = labjack.stream_analog_read:main',
            'labjack_range_data_publisher = labjack.publish_range_data:main',
            'labjack_point_publisher = labjack.publish_point:main',
            'labjack_pointcould2_publisher = labjack.publish_pointcould2:main',
            'register_implant_to_laser = grbl_ros2_gui.ros_register_implant_to_laser:main',
        ],
    },
)
