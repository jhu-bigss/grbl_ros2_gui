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
from PyQt5.QtCore import pyqtSlot

import rclpy


class Backend(QtCore.QObject):


    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('grbl')
        self.node.get_logger().info('Declaring ROS parameters')
        self.node.declare_parameters(
            namespace='',
            parameters=[
                ('port', None),
                ('baudrate', 0),
            ])

        self.shutdown_requested = False

        # self.lock = threading.Lock()

    def spin(self):
        while rclpy.ok() and not self.shutdown_requested:
            # DO blah blah blah

            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_node()

    @pyqtSlot(object)
    def set_ros_parameters(self, list_params):
        self.node.set_parameters(list_params)

    def terminate_ros_spinner(self):
        self.node.get_logger().info("shutdown requested [ROS]")
        self.shutdown_requested = True