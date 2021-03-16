import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32

import u3

# scan frequency in Hz
DEFAULT_SCAN_FREQUENCY = 10000

class LabjackPublisherNode(Node):

    def __init__(self, channel=0, scan_freq=DEFAULT_SCAN_FREQUENCY):
        super().__init__('labjack_publisher')
        self.channel_name = "AIN"+str(channel)
        # Device U3
        self.d = u3.U3()
        # To learn the if the U3 is an HV
        self.d.configU3()
        # For applying the proper calibration to readings
        self.d.getCalibrationData()
        # Set the FIO0 to Analog (d1 = b00000001)
        self.d.configIO(FIOAnalog=1)

        self.get_logger().info("Configuring U3 Stream")
        self.d.streamConfig(NumChannels=1, PChannels=[0], NChannels=[31], Resolution=3, ScanFrequency=scan_freq)
        if self.d is None:
            return
        # Start streaming
        self.d.streamStart()

        self.publisher_ain0 = self.create_publisher(Float32, 'labjack_ain0', QoSProfile(depth=10))
        timer_period = 1.0/scan_freq  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        # Conversion in milli volt for gain 1
        data = next(self.d.streamData())
        if data is not None:
            msg.data = sum(data[self.channel_name])/len(data[self.channel_name])
            self.get_logger().info('Analog Read: "{0}"'.format(msg.data))
            self.publisher_ain0.publish(msg)

    def labjack_stop_stream(self):
        self.d.streamStop
        self.d.close()


def main(args=None):
    rclpy.init(args=args)

    labjack_pub_node = LabjackPublisherNode()

    rclpy.spin(labjack_pub_node)

    labjack_pub_node.labjack_stop_stream()
    labjack_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()