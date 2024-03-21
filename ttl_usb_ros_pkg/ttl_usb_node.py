import serial
import serial.tools.list_ports
import time
import struct

import rclpy
from rclpy.node import Node

import gps

from sensor_msgs.msg import NavSatFix, NavSatStatus

class TTLUsbNode(Node):
    def __init__(self):
        super().__init__('ttl_usb')
        self.declare_parameter('gps_topic', '~/gps/navsat')
        
        topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        
        self.get_logger().info('Topic name: "%s"' % topic)
        
        self.session = gps.gps(mode=gps.WATCH_ENABLE)

        self.publisher_ = self.create_publisher(NavSatFix, topic, 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = self.process()

        if msg is None:
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher_.publish(msg)
    
    def process(self) -> NavSatFix:
        while 0 == self.session.read():
            if not (gps.MODE_SET & self.session.valid):
                # not useful, probably not a TPV message
                continue

            self.get_logger().info('Mode: %s(%d) Time: ' %
                (("Invalid", "NO_FIX", "2D", "3D")[self.session.fix.mode],
                self.session.fix.mode))
            # print time, if we have it
            if gps.TIME_SET & self.session.valid:
                self.get_logger().info(self.session.fix.time)
            else:
                self.get_logger().info('n/a')

            if ((gps.isfinite(self.session.fix.latitude) and
                gps.isfinite(self.session.fix.longitude))):
                navsatfix = NavSatFix()
                navsatfix.latitude = self.session.fix.latitude
                navsatfix.longitude = self.session.fix.longitude
                navsatfix.status = self.session.fix.mode

                return navsatfix
            else:
                return None
        
def main(args=None):
    rclpy.init(args=args)

    usb_node = TTLUsbNode()

    rclpy.spin(usb_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    usb_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
