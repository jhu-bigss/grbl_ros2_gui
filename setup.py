import os
from glob import glob
from setuptools import setup
# from setuptools import find_packages

package_name = 'grbl_ros2_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'labjack_node = labjack.labjack_node:main'
        ],
    },
)
