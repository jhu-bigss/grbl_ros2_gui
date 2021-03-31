import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Range
from std_msgs.msg import Float32

sensor_frame_name = 'sensor'

# 4-20 mA corresponds to 0.472 to 2.36 volts that represents 0.035 -+ 0.015 m
range_min = 0.02
range_max = 0.05
volts_min = 0.472
volts_max = 2.36

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
            'labjack_range',
            QoSProfile(depth=10))

    def process_data_callback(self, msg):
        r = Range()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = sensor_frame_name
        r.radiation_type = 1
        # r.field_of_view = 0.0  # X axis of the sensor
        r.min_range = range_min
        r.max_range = range_max
        r.range = range_min + (range_max - range_min) * (msg.data - volts_min)/(volts_max - volts_min)
        self.pub.publish(r)


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