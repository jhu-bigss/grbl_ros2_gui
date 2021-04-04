import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Transform, PolygonStamped, Polygon, Point32
from sensor_msgs.msg import Range, PointField, PointCloud2
from std_msgs.msg import Header

from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger

import numpy as np
from scipy.spatial.transform import Rotation as R

polygon_offset_distance = 0.035  # m

class LabjackProfilerNode(Node):

    def __init__(self):
        super().__init__('labjack_pointcloud2_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, qos=QoSProfile(depth=10))

        self.pub_polygon = self.create_publisher(
            PolygonStamped,
            'labjack_polygon',
            QoSProfile(depth=10))

        self.timer = self.create_timer(timer_period_sec=0.5, callback=self.publish_polygon_callback)

        self.sub_range = self.create_subscription(
            Range,
            'labjack_range',
            self.publish_pointcloud_callback,
            QoSProfile(depth=10))

        self.pub_pcd2 = self.create_publisher(
            PointCloud2,
            'labjack_pointcloud2',
            QoSProfile(depth=10))

        # Create a service client to update scanning parameters from grbl node for Rviz preview
        self.client = self.create_client(GetParameters, '/grbl/get_parameters')
        self.request = GetParameters.Request()
        self.request.names = ['scan_mode',
                              'scan_width',
                              'scan_height']
        # Connect to grbl srv server
        self.client.wait_for_service()

        # create service for starting/stopping the scan process
        self.srv_scan = self.create_service(Trigger, 'labjack_pointcloud2_publisher/scan_on_off', self.set_running)

        # Declare scanning parameters
        self.scan_mode = 0  # 0 is rectangular, 1 is circular
        self.scan_width = 0.0
        self.scan_height = 0.0

        self.scan_points = []
        self.current_transform = Transform()

        self.running = False

    def update_param_callback(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn("service call failed %r" % (e,))
        else:
            self.scan_mode = result.values[0].integer_value
            self.scan_width = result.values[1].double_value * 0.001  # convert mm to m
            self.scan_height = result.values[2].double_value * 0.001

    def publish_polygon_callback(self):
        polygon_stamped = PolygonStamped()
        polygon_stamped.header = Header(frame_id='sensor')
        if not self.running:
            # Update scanning parameters from grbl node
            future = self.client.call_async(self.request)
            future.add_done_callback(self.update_param_callback)
            if self.scan_mode == 0:
                polygon_stamped.polygon = self.generate_rectangular_polygon(self.scan_width, self.scan_height)
            elif self.scan_mode == 1:
                polygon_stamped.polygon = self.generate_circular_polygon(self.scan_width)
        self.pub_polygon.publish(polygon_stamped)

    def generate_rectangular_polygon(self, width, height):
        polygon_rect = Polygon()
        polygon_rect.points.append(Point32(x=polygon_offset_distance,y=height/2,z=width/2))
        polygon_rect.points.append(Point32(x=polygon_offset_distance,y=height/2,z=-width/2))
        polygon_rect.points.append(Point32(x=polygon_offset_distance,y=-height/2,z=-width/2))
        polygon_rect.points.append(Point32(x=polygon_offset_distance,y=-height/2,z=width/2))
        return polygon_rect

    def generate_circular_polygon(self, radius):
        polygon_circle = Polygon()
        theta = np.linspace(0, 2*np.pi, 150)
        a = radius * np.cos(theta)
        b = radius * np.sin(theta)
        for h, v in zip(a, b):
            polygon_circle.points.append(Point32(x=polygon_offset_distance,y=v,z=h))
        return polygon_circle

    def publish_pointcloud_callback(self, range_msg):
        try:
            trans = self.tf_buffer.lookup_transform('W', 'sensor', time=rclpy.time.Time())
        except:
            print('lookup_transform(): No transformation found')
        else:
            if self.running:
            # update current transform if machine moved
            # if self.current_transform != trans.transform:
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
                    self.scan_points.append(point_W[:3])

            # Create pointcloud2 message and publish
            pcd = self.generate_point_cloud(np.asarray(self.scan_points), 'W')
            self.pub_pcd2.publish(pcd)

    def generate_point_cloud(self, points, parent_frame):
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

    def set_running(self, request, response):
        self.running = not self.running
        response.success = True
        response.message = "Running: {}".format(self.running)
        return response

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