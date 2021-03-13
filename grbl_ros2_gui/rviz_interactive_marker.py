# Reference: https://gist.github.com/awesomebytes/2aa18ba3b821b2f580a2

import rclpy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, \
    InteractiveMarkerFeedback, InteractiveMarkerUpdate, InteractiveMarkerPose
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion
import tf2_ros


class GRBLInteractiveMarker():

    def __init__(self, node, frames):
        self.node = node
        self.name = 'interactive_marker'
        self.server = InteractiveMarkerServer(self.node, self.name)
        self.menu_handler = MenuHandler()
        self.frames = frames

        self.makeGroupIM()
        self.server.applyChanges()

    def processFeedback(self, feedback):
        """
        :type feedback: InteractiveMarkerFeedback
        """
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
            if feedback.menu_entry_id == 1:
                self.node.get_logger().debug("Jogging laser head...")
                # TODO: send jog command to grbl
                print("I got to move there!")

        # When clicking this event triggers!
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.node.get_logger().debug(s + ": pose changed")
            # TODO: move machine MPos

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.node.get_logger().debug(s + ": mouse down" + mp + ".")

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.node.get_logger().debug(s + ": mouse up" + mp + ".")

        self.server.applyChanges()

    def make_marker_laser_head(self):
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.pose.position.z = 0.069/2
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.069
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def make_marker_rotary_a(self):
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 0.03 # length
        marker.scale.y = 0.03 # width
        marker.scale.z = 0.069 # arrow height
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker

    def make_interactive_marker_laser_head(self):
        int_marker_laser_head = InteractiveMarker()
        int_marker_laser_head.header.frame_id = self.frames[-1]
        int_marker_laser_head.pose = Pose()
        int_marker_laser_head.scale = 0.1
        int_marker_laser_head.name = self.frames[-1]
        int_marker_laser_head.description = "interactive marker attached to " + self.frames[-1]

        # insert laser head
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_marker_laser_head())
        int_marker_laser_head.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker_laser_head.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker_laser_head.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_xy"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker_laser_head.controls.append(control)

        return int_marker_laser_head

    def make_interactive_marker_rotary_a(self):
        int_marker_rotary_a = InteractiveMarker()
        int_marker_rotary_a.header.frame_id = self.frames[0]
        int_marker_rotary_a.pose = Pose()
        int_marker_rotary_a.scale = 0.1
        int_marker_rotary_a.name = self.frames[0]
        int_marker_rotary_a.description = "interactive marker attached to " + self.frames[0]

        # insert laser head
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_marker_rotary_a())
        int_marker_rotary_a.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker_rotary_a.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker_rotary_a.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_xy"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker_rotary_a.controls.append(control)

        return int_marker_rotary_a

    def makeGroupIM(self):
        int_marker_laser_head = self.make_interactive_marker_laser_head()
        # TODO: add rotary_a interactive marker
        self.server.insert(int_marker_laser_head, feedback_callback=self.processFeedback)

        self.menu_handler.insert("Jog here", callback=self.processFeedback)
        self.menu_handler.apply(self.server, int_marker_laser_head.name)

    def terminate_interactive_marker_server(self):
        self.server.shutdown()