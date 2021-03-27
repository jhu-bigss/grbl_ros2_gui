import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Range
from std_msgs.msg import Float32


class LabjackProcessDataNode(Node):

    def __init__(self):
        super().__init__('labjack_process_data_node')
        self.sub = self.create_subscription(
            Float32,
            'labjack_ain0',
            self.process_data_callback,
            QoSProfile(depth=10))
        self.pub = self.create_publisher(
            Range,
            'range',
            QoSProfile(depth=10))
        self.seq_id = 0

    def process_data_callback(self, msg):
        r = Range()
        r.header.seq = self.seq_id
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = 'T'  # should be the frame name of the sensor
        r.radiation_type = 1
        r.field_of_view = 0  # X axis of the sensor
        r.min_range = 10
        r.max_range = 30
        r.range = msg.data
        self.pub.publish(r)
        self.seq_id += 1


def main(args=None):

    rclpy.init(args=args)
    labjack_process_data_node = LabjackProcessDataNode()
    rclpy.spin(labjack_process_data_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    labjack_process_data_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()