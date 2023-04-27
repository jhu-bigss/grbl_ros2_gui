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

from math import radians

class Backend(QtCore.QObject):

    sig_push_gcode = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()

        qos_profile = QoSProfile(depth=10)
        self.node = rclpy.create_node('grbl_ros2_gui')
        self.node.get_logger().info('Declaring ROS parameters')
        self.node.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyACM0'),
                ('baudrate', 115200),
                ('jogSpeed', 300.0),
                ('max_travel_x', 280.0),
                ('max_travel_y', 280.0),
                ('max_travel_z', 65.0),
                ('max_travel_a', 150.0),
                ('max_travel_b', 0.0)
            ])

        self.joint_pub = self.node.create_publisher(JointState, 'joint_states', qos_profile)
        self.joint_states = JointState()

        # RViz interactive markers for jogging
        self.rviz_interactive_markers = GRBLInteractiveMarker(self.node)
        self.rviz_interactive_markers.sig_jog_axis.connect(self.sig_push_gcode)

        self.cmd_sub = self.node.create_subscription(String, 'cmd/gcode', self.push_gcode, qos_profile)

        self.shutdown_requested = False
        self.node.get_logger().info("{0} started".format(self.node.get_name()))
        # self.lock = threading.Lock()

    def spin(self):
        while rclpy.ok() and not self.shutdown_requested:

            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_node()

    @pyqtSlot(str)
    def update_status(self, status_msg : str):
        
        if status_msg[0] != "<" or status_msg[-1] != ">":
            return print("grbl status_msg error ! \n[{}] Incorrect status.").format(status_msg)

        # Parse status message
        machine_position = status_msg[1:-1].split("|")[1]
        machine_position = machine_position[5:].split(",")
        joint_values = [0.001*float(i) for i in machine_position[:3]] + [radians(float(i)) for i in machine_position[3:]]
        joint_values[1] = -joint_values[1]
        joint_values[2] = -joint_values[2]

        # Publish joints states to ROS
        self.joint_states.header.stamp = self.node.get_clock().now().to_msg()
        self.joint_states.name = ["X", "Y", "Z", "A", "B"]
        self.joint_states.position = joint_values
        self.joint_pub.publish(self.joint_states)

    @pyqtSlot(float)
    def update_jogSpeed(self, value : float):
        self.set_ros_parameters([rclpy.parameter.Parameter('jogSpeed', rclpy.Parameter.Type.DOUBLE, value)])

    def set_ros_parameters(self, params: list):
        self.node.set_parameters(params)

    def push_gcode(self, gcode):
        self.sig_push_gcode.emit(gcode.data)

    def terminate_ros_backend(self):
        self.node.get_logger().info("shutdown...")
        self.rviz_interactive_markers.terminate_interactive_marker_server()
        self.shutdown_requested = True