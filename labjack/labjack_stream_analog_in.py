import atexit

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32
from std_srvs.srv import Trigger

import u3

# From table 3.2-1 with resolutions and cognate max stream scan frequencies.
# Maximum scan frequencies are in samples/s (shared across all channels).
# https://labjack.com/support/datasheets/u3/operation/stream-mode
resolution_index2max_scan_freq = {
    0: 2500,
    1: 10000,
    2: 20000,
    3: 50000
}

class LabjackPublisher(Node):

    def __init__(self, channel=0, res_index=3):
        super().__init__('labjack_publisher')
        scan_freq=resolution_index2max_scan_freq[res_index]
        # Exit handler
        atexit.register(self.destroy_node)
        # Anglog 0 Publisher
        self.pub_ain0 = self.create_publisher(Float32, 'labjack_ain0', QoSProfile(depth=10))

        self.channel_name = "AIN"+str(channel)
        # Device U3
        self.d = u3.U3()
        # To learn the if the U3 is an HV
        self.d.configU3()
        # For applying the proper calibration to readings
        self.d.getCalibrationData()
        # Set the FIO0 to Analog (d1 = b00000001)
        self.d.configIO(FIOAnalog=1)
        # register atexit handlers
        atexit.register(self.d.close)

        self.get_logger().info("Configuring U3 Streaming")
        self.d.streamConfig(NumChannels=1, PChannels=[0], NChannels=[31], Resolution=res_index, ScanFrequency=scan_freq)
        if self.d is None:
            return
        # Start streaming
        self.d.streamStart()
        atexit.register(self.d.streamStop)

        timer_period = 1.0/scan_freq  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        # Conversion in milli volt for gain 1
        data = next(self.d.streamData())
        if data is not None:
            msg.data = sum(data[self.channel_name])/len(data[self.channel_name])
            self.get_logger().debug('Analog Read: "{0}"'.format(msg.data))
            self.pub_ain0.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    labjack_pub_node = LabjackPublisher()
    rclpy.spin(labjack_pub_node)
    labjack_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()