import serial
import serial.tools.list_ports
import serial
import time
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus

class TTLUsbNode(Node):
    def __init__(self):
        super().__init__('ttl_usb')
        self.declare_parameter('gps_topic', '~/gps/navsat')
        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        self.declare_parameter('gps_id', 'GGA')
        
        topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        port = self.get_parameter('usb_port').get_parameter_value().string_value
        self.gps_id = self.get_parameter('gps_id').get_parameter_value().string_value
        
        self.get_logger().info('Topic name: "%s"' % topic)
        self.get_logger().info('Selected port: "%s"' % port)
        self.get_logger().info('Selected GPS message: "%s"' % self.gps_id)
        
        self.UTCoffset=int(-time.timezone/3600) # calculate UTC offset
        self.port = None
        try:
            self.port = serial.Serial(port)
            self.port.close()
            print(self.port)
        except (OSError, serial.SerialException):
            self.get_logger().error("Cannot open serial port!")
        
        self.port.open()
        self.publisher_ = self.create_publisher(NavSatFix, topic, 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        data_bare = self.port.readline()
        data = data_bare.decode('utf-8').rstrip()
        msg = self.parseData(data, data_bare)
        if msg is None:
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher_.publish(msg)
    
    # function for repairing time format AKA adding ":" to time string and time zone offset
    # input: 123456 
    # output: 12+XX:34:56 [local time]
    # format: [hh:mm:ss] 
    def repairTime(self, time):
        return (str(int(time[:2])+self.UTCoffset)) + ":" + time[2:4] + ":" + time[4:6]

    # function for parsing data from GPS module
    def parseData(self, receivedData, receivedDataBytes) -> NavSatFix:
        receivedData = str(receivedData)
        self.get_logger().debug(f"Received: {receivedDataBytes}")
        # check if data is available
        if len(receivedData) < 1:
            self.get_logger().debug("No data received")
            return 
        # check if data is not too long
        if len(receivedData) > 82:
            self.get_logger().debug("Data too long")
            return 
        # check if data starts with $ 
        if receivedData[0] != "$":
            self.get_logger().debug("Invalid data - no $ at the beginning")
            return

        receivedChecksum = receivedData.split("*")[1]  # get checksum
        data = receivedData.split("*")[0]  # remove checksum from data
        data = data.split(",")  # split data into a list of shorted string
        data.append(receivedChecksum)  # add checksum to the end of the list
        
        navsatfix = NavSatFix()

        # $XXYYY,ZZ,ZZ,ZZ,ZZ,ZZ,ZZ,ZZ,ZZ,ZZ,ZZ,ZZ,ZZ,ZZ,ZZ,ZZ*CC
        # $ - header
        # XX - talker ID
        # YYY - message ID
        # ZZ - data
        # CC - checksum

        talkerID = data[0][1:3]  # get talker ID
        if   talkerID == "GP":          _stalkerID = "GPS"
        elif talkerID == "GL":          _stalkerID = "GLONASS"
        elif talkerID == "GN":          _stalkerID = "GNSS"
        elif talkerID == "GA":          _stalkerID = "Galileo"
        elif talkerID == ("GB" or "GB"):_stalkerID = "BeiDou"
        elif talkerID == "QZ":          _stalkerID = "QZSS"
        elif talkerID == "NA":          _stalkerID = "NavIC"
        else:                           _stalkerID = "Unknown"
        self.get_logger().debug("Talker ID: %s" % (_stalkerID))

        messageID = data[0][3:6]  # get message ID
        self.get_logger().debug("Message ID: %s" % (messageID))

        # GPRMC - Recommended Minimum Specific GPS/TRANSIT Data
        if (messageID == self.gps_id) and ("RMC" == messageID):
            self.get_logger().debug("Time: %s" % (self.repairTime(data[1])))
            status = ["active", "No fix avaibile", "invalid"]
            if   data[2] == "A": _sstatus = status[0]
            elif data[2] == "V": _sstatus = status[1]
            else:                _sstatus = status[2]
            self.get_logger().debug("Status: %s" % (_sstatus))
            self.get_logger().debug("Latitude: %s %s" % (data[3], data[4]))
            self.get_logger().debug("Longitude: %s %s" % (data[5], data[6]))
            self.get_logger().debug("Speed in knots: %s " % (data[7]))
            self.get_logger().debug("Track Angle: %s" % (data[8]))
            self.get_logger().debug("Date: %s" % (data[9]))
            self.get_logger().debug("Magnetic Variation: %s" % (data[10]))
            self.get_logger().debug("Mag Var Direction: %s" % (data[11]))
            self.get_logger().debug("Checksum: %s" % (data[12]))
            navsatfix.latitude = data[3], data[4]

        # Geographic Position - Latitude/Longitude
        elif (messageID == self.gps_id) and ("GLL" == messageID):
            self.get_logger().debug("Latitude: %s" % (data[1]))
            self.get_logger().debug("Longitude: %s" % (data[3]))
            self.get_logger().debug("Time: %s" % (data[5]))
            GLLstatus = ["active", "inactive", "invalid"]
            if   data[6] == "A": _sGLLstatus = GLLstatus[0]
            elif data[6] == "V": _sGLLstatus = GLLstatus[1]
            else:                _sGLLstatus = GLLstatus[2]
            self.get_logger().debug("Status: %s" % (_sGLLstatus))
            self.get_logger().debug("Checksum: %s" % (data[7]))

        # GPGGA - Global Positioning System Fix Data
        elif (messageID == self.gps_id) and ("GGA" == messageID):
            self.get_logger().debug("Time: %s" % (data[1]))
            self.get_logger().info("Latitude: %s%s" % (data[2], data[3]))
            self.get_logger().info("Longitude: %s%s" % (data[4], data[5]))
            fixDetails = [
                "0. Invalid, no position available.",
                "1. Autonomous GPS fix, no correction data used.",
                "2. DGPS fix, using a local DGPS base station or correction service such as WAAS or EGNOS.",
                "3. PPS fix, I’ve never seen this used.",
                "4. RTK fix, high accuracy Real Time Kinematic.",
                "5. RTK Float, better than DGPS, but not quite RTK.",
                "6. Estimated fix (dead reckoning).",
                "7. Manual input mode.",
                "8. Simulation mode.",
                "9. WAAS fix",
            ]
            self.get_logger().info("Fix Quality: %s" % (data[6]))
            self.get_logger().debug("Fix Quality Details: %s" % (fixDetails[int(data[6])]))
            self.get_logger().info("Number of Satellites: %s" % (data[7]))
            HDOP = data[8]  # Horizontal Dilution of Precision = accuracy of horizontal position
            self.get_logger().info("Horizontal Dilution of Precision: %s" % (HDOP))
            if HDOP == "0": self.get_logger().debug("No HDOP available")
            if HDOP >= "6": self.get_logger().debug("HDOP is terrible")
            self.get_logger().debug("Altitude ASL: %s %s" % (data[9], data[10]))
            self.get_logger().debug("Height of Geoid: %s %s" % (data[11], data[12]))
            self.get_logger().debug("Time since last DGPS update: %s" % (data[13]))
            self.get_logger().debug("DGPS reference station id: %s" % (data[14]))
            self.get_logger().debug("Checksum: %s" % (data[15]))
            # lat = bytearray()
            # lat.extend()
            # lat.extend() 
            navsatfix.status.status = int(int(data[6]) > 0)
            if navsatfix.status.status == 0:
                return navsatfix
            self.get_logger().info("Checksum: %s" % ((data[2]+data[3]).encode('utf-8')))
            navsatfix.latitude = struct.unpack('f', (data[2]+data[3]).encode('utf-8'))
            navsatfix.longitude = struct.unpack('f', (data[4]+data[5]).encode('utf-8'))
            
            navsatfix.altitude = struct.unpack('f', (data[9] + data[10]).encode('utf-8'))

        # abort if unknown header
        else:
            self.get_logger().debug("No handled message ID")
            return None

        # check checksum
        if self.checkChecksum(receivedData[1:]):  # remove $ from cropped data, because checkChecksum function does not need it
            self.get_logger().debug("Checksum OK")
        else:
            self.get_logger().debug("Checksum Invalid or Unavailable")
            return None
        
        return navsatfix


    # function for calculating checksum and comparing it with received one
    def checkChecksum(self, word):
        # Split the message into the message body and the checksum
        message, checksum = word.split("*")

        # Calculate the expected checksum
        expected_checksum = 0
        for char in message:
            expected_checksum ^= ord(char)

        # Compare the expected checksum to the actual checksum
        actual_checksum = int(checksum, 16)
        if expected_checksum == actual_checksum:
            return True
        else:
            return False
        
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
