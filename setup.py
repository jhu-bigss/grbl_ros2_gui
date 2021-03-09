from setuptools import setup

package_name = 'grbl_ros2_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua Liu',
    maintainer_email='liushuya7@gmail.com',
    description='A ROS2 GUI for interfacing with grbl devices',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grbl_gui = grbl_ros2_gui.grbl_gui:main'
        ],
    },
)
