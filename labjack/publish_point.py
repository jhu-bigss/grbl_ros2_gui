import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Transform, PointStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Header

from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger

import numpy as np
from scipy.spatial.transform import Rotation as R

polygon_offset_distance = 0.035  # m

class LabjackProbeNode(Node):

    def __init__(self):
        super().__init__('labjack_point_publisher')

        # create a TransformListener object. Once the listener is created, it starts receiving tf2 transformations over the wire, and buffers them for up to 10 seconds.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, qos=QoSProfile(depth=10))

        self.pub_point = self.create_publisher(
            PointStamped,
            'labjack_point',
            QoSProfile(depth=10))

        self.sub_range = self.create_subscription(
            Range,
            'labjack_range',
            self.publish_point_callback,
            QoSProfile(depth=10))

    def publish_point_callback(self, range_msg):
        try:
            trans = self.tf_buffer.lookup_transform('W', 'sensor', time=rclpy.time.Time())
        except:
            print('lookup_transform(): No transformation found')
        else:
            # Check if range values < range_min or > range_max
            if range_msg.range <= range_msg.max_range and range_msg.range >= range_msg.min_range:
                point_S = np.array([range_msg.range, 0.0, 0.0, 1])

                # Homogeneous transformation from W to Sensor
                H_trans_W__S = np.eye(4)
                H_trans_W__S[:3,:3] = R.from_quat([
                                            self.current_transform.rotation.x,
                                            self.current_transform.rotation.y,
                                            self.current_transform.rotation.z,
                                            self.current_transform.rotation.w
                                            ]).as_matrix()
                H_trans_W__S[0,3] = self.current_transform.translation.x
                H_trans_W__S[1,3] = self.current_transform.translation.y
                H_trans_W__S[2,3] = self.current_transform.translation.z

                # Transform the measured point, add to the point cloud
                point_W = H_trans_W__S.dot(point_S)
                
                # Publish the point
                point_stamped = PointStamped()
                point_stamped.header.frame_id = 'W'
                point_stamped.header.stamp = self.get_clock().now().to_msg()
                point_stamped.point.x = point_W[0]
                point_stamped.point.y = point_W[1]
                point_stamped.point.z = point_W[2]
                self.pub_point.publish(point_stamped)


def main(args=None):

    rclpy.init(args=args)
    labjack_probe_node = LabjackProbeNode()
    rclpy.spin(labjack_probe_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    labjack_probe_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()