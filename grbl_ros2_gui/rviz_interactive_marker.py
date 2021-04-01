# Reference: https://gist.github.com/awesomebytes/2aa18ba3b821b2f580a2

from PyQt5 import QtCore

import rclpy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, \
    InteractiveMarkerFeedback, InteractiveMarkerUpdate, InteractiveMarkerPose
from geometry_msgs.msg import Pose
import math

class GRBLInteractiveMarker(QtCore.QObject):

    sig_jog_axis = QtCore.pyqtSignal(str)

    def __init__(self, node):
        super().__init__()

        self.node = node
        self.name = 'interactive_marker'
        self.server = InteractiveMarkerServer(self.node, self.name)
        self.menu_handler = MenuHandler()

        self.makeGroupIM()
        self.server.applyChanges()

    def processFeedback(self, feedback):
        """
        :type feedback: InteractiveMarkerFeedback
        """
        JOG = 1
        RESET = 2
        current_interactive_marker = self.server.get(feedback.marker_name)
        current_pose = current_interactive_marker.pose

        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"
        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.node.get_logger().debug(s + ": button click" + mp + ".")

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self.node.get_logger().debug(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
            if feedback.menu_entry_id == JOG:
                self.node.get_logger().debug("Jogging...")
                # Reset interactive marker
                self.server.setPose(feedback.marker_name, Pose())
                # Send jog command to grbl
                if feedback.marker_name == 'laser_xy':
                    jog_x = current_pose.position.x * 1000
                    jog_y = current_pose.position.y * 1000
                    self.grbl_jog_axis(['X','Y'], [jog_x, jog_y])
                elif feedback.marker_name == 'laser_z':
                    jog_z = - current_pose.position.z * 1000  # flip sign
                    self.grbl_jog_axis('Z', jog_z)
                elif feedback.marker_name == 'rotary_a':
                    x = current_pose.orientation.x
                    y = current_pose.orientation.y
                    z = current_pose.orientation.z
                    w = current_pose.orientation.w
                    euler_angle = self.euler_from_quaternion(x, y, z, w)
                    jog_a = math.degrees(euler_angle[0])
                    self.grbl_jog_axis('A', jog_a)
                elif feedback.marker_name == 'rotary_b':
                    x = current_pose.orientation.x
                    y = current_pose.orientation.y
                    z = current_pose.orientation.z
                    w = current_pose.orientation.w
                    euler_angle = self.euler_from_quaternion(x, y, z, w)
                    jog_b = math.degrees(euler_angle[-1])
                    self.grbl_jog_axis('B', jog_b)
            if feedback.menu_entry_id == RESET:
                self.node.get_logger().debug("reset interactive marker pose.")
                # Reset interactive marker
                self.server.setPose(feedback.marker_name, Pose())

        # When clicking this event triggers!
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.node.get_logger().debug(s + ": pose changed")
            # TODO: check if the pose exceeds the joint limit
            # print(current_pose)

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.node.get_logger().debug(s + ": mouse down" + mp + ".")

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.node.get_logger().debug(s + ": mouse up" + mp + ".")

        self.server.applyChanges()

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z  # in radians

    def grbl_jog_axis(self, jog_axis_name: str, jog_value: float):
        # Jog by relative positioning
        gcode = "$J=G91"
        if type(jog_axis_name) == list:
            for axis_name, val in zip(jog_axis_name, jog_value):
                gcode += "{}{:0.3f}".format(axis_name, val)
        else:
            gcode += "{}{:0.3f}".format(jog_axis_name, jog_value)
        # Jog Speed mm/min
        param_jog_speed = self.node.get_parameter('jog_speed')
        gcode += "F{:0.2f}".format(param_jog_speed.value)
        
        # Emit the gcode to grbl
        self.sig_jog_axis.emit(gcode)

    def make_marker_laser_xy(self):
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.pose.position.z = -0.069/2
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.069
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8  # Transparency

        return marker

    def make_marker_laser_z(self):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.pose.position.z = -0.0015
        marker.scale.x = 0.43
        marker.scale.y = 0.20
        marker.scale.z = 0.003
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8

        return marker

    def make_marker_rotary_a(self):
        marker = Marker()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://grbl_ros2_gui/meshes/rotary_middle_link.STL"
        marker.mesh_use_embedded_materials = False
        marker.color.r = 0.50196
        marker.color.g = 0.50196
        marker.color.b = 0.50196
        marker.color.a = 0.8

        return marker

    def make_marker_rotary_b(self):
        marker = Marker()
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://grbl_ros2_gui/meshes/rotary_top_link.STL"
        marker.mesh_use_embedded_materials = False
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.8

        return marker

    def make_interactive_marker_laser_xy(self):
        int_marker_laser_xy = InteractiveMarker()
        int_marker_laser_xy.header.frame_id = 'T'
        # int_marker_laser_xy.pose = Pose()
        int_marker_laser_xy.scale = 0.06
        int_marker_laser_xy.name = 'laser_xy'
        int_marker_laser_xy.description = "interactive marker " + int_marker_laser_xy.name

        # insert laser head
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_marker_laser_xy())
        int_marker_laser_xy.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker_laser_xy.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker_laser_xy.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_xy"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker_laser_xy.controls.append(control)

        return int_marker_laser_xy

    def make_interactive_marker_laser_z(self):
        int_marker_laser_z = InteractiveMarker()
        int_marker_laser_z.header.frame_id = 'base_link'
        # int_marker_laser_z.pose = Pose()
        int_marker_laser_z.scale = 0.06
        int_marker_laser_z.name = 'laser_z'
        int_marker_laser_z.description = "interactive marker " + int_marker_laser_z.name

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_marker_laser_z())
        int_marker_laser_z.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker_laser_z.controls.append(control)

        return int_marker_laser_z

    def make_interactive_marker_rotary_a(self):
        int_marker_rotary_a = InteractiveMarker()
        int_marker_rotary_a.header.frame_id = 'rotary_middle'
        # int_marker_rotary_a.pose = Pose()
        int_marker_rotary_a.scale = 0.08
        int_marker_rotary_a.name = 'rotary_a'
        int_marker_rotary_a.description = "interactive marker " + int_marker_rotary_a.name

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_marker_rotary_a())
        int_marker_rotary_a.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "rotate_a"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker_rotary_a.controls.append(control)

        return int_marker_rotary_a

    def make_interactive_marker_rotary_b(self):
        int_marker_rotary_b = InteractiveMarker()
        int_marker_rotary_b.header.frame_id = 'W'
        # int_marker_rotary_b.pose = Pose()
        int_marker_rotary_b.scale = 0.07
        int_marker_rotary_b.name = 'rotary_b'
        int_marker_rotary_b.description = "interactive marker " + int_marker_rotary_b.name

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_marker_rotary_b())
        int_marker_rotary_b.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_b"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker_rotary_b.controls.append(control)

        return int_marker_rotary_b

    def makeGroupIM(self):
        int_marker_laser_xy = self.make_interactive_marker_laser_xy()
        int_marker_laser_z = self.make_interactive_marker_laser_z()
        int_marker_rotary_a = self.make_interactive_marker_rotary_a()
        int_marker_rotary_b = self.make_interactive_marker_rotary_b()
        self.server.insert(int_marker_laser_xy, feedback_callback=self.processFeedback)
        self.server.insert(int_marker_laser_z, feedback_callback=self.processFeedback)
        self.server.insert(int_marker_rotary_a, feedback_callback=self.processFeedback)
        self.server.insert(int_marker_rotary_b, feedback_callback=self.processFeedback)

        self.menu_handler.insert("Jog Here", callback=self.processFeedback)
        self.menu_handler.insert("Reset", callback=self.processFeedback)
        self.menu_handler.apply(self.server, int_marker_laser_xy.name)
        self.menu_handler.apply(self.server, int_marker_laser_z.name)
        self.menu_handler.apply(self.server, int_marker_rotary_a.name)
        self.menu_handler.apply(self.server, int_marker_rotary_b.name)

    def terminate_interactive_marker_server(self):
        self.server.shutdown()