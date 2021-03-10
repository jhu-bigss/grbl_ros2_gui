##############################################################################
# Documentation
##############################################################################
"""
Ros backend for the GUI.
"""
##############################################################################
# Imports
##############################################################################

from PyQt5 import QtCore

import rclpy


class Backend(QtCore.QObject):


    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('grbl')
        self.node.get_logger().info('Declaring ROS parameters')
        self.node.declare_parameters(
            namespace='',
            parameters=[
                ('machine_id', None),
                ('port', None),
                ('baudrate', None),
                ('acceleration', None),  # mm / min^2
                ('x_max', None),  # mm
                ('y_max', None),  # mm
                ('z_max', None),  # mm
                ('default_v', None),  # mm / min
                ('x_max_v', None),  # mm / min
                ('y_max_v', None),  # mm / min
                ('z_max_v', None),  # mm / min
                ('x_steps', None),  # mm
                ('y_steps', None),  # mm
                ('z_steps', None),  # mm
            ])

        self.shutdown_requested = False

        # self.lock = threading.Lock()

    def spin(self):
        while rclpy.ok() and not self.shutdown_requested:
            # DO blah blah blah

            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_node()

    def terminate_ros_spinner(self):
        self.node.get_logger().info("shutdown requested [backend]")
        self.shutdown_requested = True