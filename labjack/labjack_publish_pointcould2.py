import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Transform
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud2

# from std_srvs.srv import Trigger

import numpy as np
import ros2_numpy as rnp


class LabjackProfilerNode(Node):

    def __init__(self):
        super().__init__('labjack_pointcloud2_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, qos=QoSProfile(depth=10))

        self.sub = self.create_subscription(
            Range,
            'labjack_range',
            self.generate_pointcloud_callback,
            QoSProfile(depth=10))

        self.pub = self.create_publisher(
            PointCloud2,
            'labjack_pointcloud2',
            QoSProfile(depth=10))

        self.current_transform = Transform()
        self.pointcloud_data = np.zeros(100, dtype=[
                                ('x', np.float32),
                                ('y', np.float32),
                                ('vectors', np.float32, (3,))
                                ])
        self.pointcloud_data['x'] = np.arange(100)
        self.pointcloud_data['y'] = self.pointcloud_data['x']*2
        self.pointcloud_data['vectors'] = np.arange(100)[:,np.newaxis]

    def generate_pointcloud_callback(self, range_msg):
        try:
            trans = self.tf_buffer.lookup_transform('W', 'sensor', time=rclpy.time.Time())
        except:
            print('lookup_transform(): No transformation found')
        else:
            # update current transform if machine moved
            if self.current_transform != trans.transform:
                self.current_transform = trans.transform
                point_in_T = np.array([range_msg.range, 0.0, 0.0, 1])
                H_trans_W__T = rnp.numpify(self.current_transform)
                point_in_W = H_trans_W__T.dot(point_in_T)

                # print(point_in_W)

            msg = rnp.msgify(PointCloud2, self.pointcloud_data, stamp=self.get_clock().now().to_msg(), frame_id='W')
            self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    labjack_profiler_node = LabjackProfilerNode()
    rclpy.spin(labjack_profiler_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    labjack_profiler_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()