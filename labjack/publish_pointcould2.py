import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Transform
from sensor_msgs.msg import Range, PointField, PointCloud2
from std_msgs.msg import Header

# from std_srvs.srv import Trigger

import numpy as np
from scipy.spatial.transform import Rotation as R


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
        self.points = []

    def generate_pointcloud_callback(self, range_msg):
        try:
            trans = self.tf_buffer.lookup_transform('W', 'sensor', time=rclpy.time.Time())
        except:
            print('lookup_transform(): No transformation found')
        else:
            # update current transform if machine moved
            if self.current_transform != trans.transform:
                self.current_transform = trans.transform
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
                    self.points.append(point_W[:3])

            # Create pointcloud2 message and publish
            pcd = self.point_cloud(np.asarray(self.points), 'W')
            self.pub.publish(pcd)

    def point_cloud(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
        References:
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
            http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
        """
        # In a PointCloud2 message, the point cloud is stored as an byte 
        # array. In order to unpack it, we also include some parameters 
        # which desribes the size of each individual point.

        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = Header(frame_id=parent_frame)

        return PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )

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