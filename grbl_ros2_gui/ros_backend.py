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
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from .rviz_interactive_marker import GRBLInteractiveMarker
from std_msgs.msg import String

class Backend(QtCore.QObject):

    sig_push_gcode = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()

        qos_profile = QoSProfile(depth=10)
        self.node = rclpy.create_node('grbl')
        self.node.get_logger().info('Declaring ROS parameters')
        self.node.declare_parameters(
            namespace='',
            parameters=[
                ('port', None),
                ('baudrate', 0),
                ('jog_speed', 0.0),
                ('max_travel_x', 0.0),
                ('max_travel_y', 0.0),
                ('max_travel_z', 0.0),
                ('max_travel_a', 0.0),
                ('max_travel_b', 0.0),
                ('scan_mode', 0),
                ('scan_width', 0.0),
                ('scan_height', 0.0),
                ('scan_resolution', 0.0),
                ('scan_speed', 0.0)
            ])
        self.joint_pub = self.node.create_publisher(JointState, 'joint_states', qos_profile)
        self.joint_states = JointState()

        self.rviz_interactive_markers = GRBLInteractiveMarker(self.node)
        self.rviz_interactive_markers.sig_jog_axis.connect(self.sig_push_gcode)

        self.cmd_sub = self.node.create_subscription(String, 'cmd/gcode', self.push_gcode, qos_profile)

        self.shutdown_requested = False
        self.node.get_logger().info("{0} node started".format(self.node.get_name()))
        # self.lock = threading.Lock()

    def spin(self):
        while rclpy.ok() and not self.shutdown_requested:

            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_node()

    @pyqtSlot(object,object)
    def publish_joint_states(self, joint_names, joint_values):
        self.joint_states.header.stamp = self.node.get_clock().now().to_msg()
        self.joint_states.name = joint_names
        self.joint_states.position = joint_values
        self.joint_pub.publish(self.joint_states)

    @pyqtSlot(object)
    def set_ros_parameters(self, list_params):
        self.node.set_parameters(list_params)

    def push_gcode(self, gcode):
        self.sig_push_gcode.emit(gcode.data)

    def terminate_ros_backend(self):
        self.node.get_logger().info("shutdown requested [ROS]")
        self.rviz_interactive_markers.terminate_interactive_marker_server()
        self.shutdown_requested = True