import sys
import logging
import serial
import serial.tools.list_ports
logging.basicConfig(stream=sys.stderr, level=logging.DEBUG) # enable debug logging avaibile DEBUG, INFO, WARNING, ERROR, CRITICAL
import serial
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

class TTLUsbNode(Node):
    def __init__(self):
        super().__init__('ttl_usb')
        self.declare_parameter('gps_topic', '~/gps/navsat')
        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        
        topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        port = self.get_parameter('usb_port').get_parameter_value().string_value
        
        self.get_logger().info('Topic name: "%s"' % topic)
        self.get_logger().info('Selected port: "%s"' % port)
        
        self.UTCoffset=int(-time.timezone/3600) # calculate UTC offset
        self.port = None
        try:
            self.port = serial.Serial(port)
            self.port.close()
            print(self.port)
        except (OSError, serial.SerialException):
            pass
        
        
        self.port.open()
        self.publisher_ = self.create_publisher(NavSatFix, topic, 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        data = self.port.readline().decode('utf-8').rstrip()
        
        self.parseData(data)
        
        self.publisher_.publish(msg)
    
    # function for repairing time format AKA adding ":" to time string and time zone offset
    # input: 123456 
    # output: 12+XX:34:56 [local time]
    # format: [hh:mm:ss] 
    def repairTime(self, time):
        return (str(int(time[:2])+self.UTCoffset)) + ":" + time[2:4] + ":" + time[4:6]

    # function for parsing data from GPS module
    def parseData(self, receivedData) -> NavSatFix:
        receivedData = str(receivedData)
        # check if data is available
        if len(receivedData) < 1:
            logging.debug("No data")
            return 
        # check if data is not too long
        if len(receivedData) > 82:
            logging.error("Data too long")
            return 
        # check if data starts with $ 
        if receivedData[0] != "$":
            logging.error("Invalid data - no $ at the beginning")
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
        logging.info("Talker ID: %s" % (_stalkerID))

        messageID = data[0][3:6]  # get message ID
        logging.info("Message ID: %s" % (messageID))

        # GPRMC - Recommended Minimum Specific GPS/TRANSIT Data
        if messageID == "RMC":
            logging.info("Time: %s" % (self.repairTime(data[1])))
            status = ["active", "No fix avaibile", "invalid"]
            if   data[2] == "A": _sstatus = status[0]
            elif data[2] == "V": _sstatus = status[1]
            else:                _sstatus = status[2]
            logging.info("Status: %s" % (_sstatus))
            logging.info("Latitude: %s %s" % (data[3], data[4]))
            logging.info("Longitude: %s %s" % (data[5], data[6]))
            logging.info("Speed in knots: %s " % (data[7]))
            logging.info("Track Angle: %s" % (data[8]))
            logging.info("Date: %s" % (data[9]))
            logging.info("Magnetic Variation: %s" % (data[10]))
            logging.info("Mag Var Direction: %s" % (data[11]))
            logging.info("Checksum: %s" % (data[12]))
            navsatfix.latitude = data[3], data[4]
            

        # detailed GPS Satellites in View
        elif messageID == "GSV":
            logging.info("Number of messages: %s" % (data[1]))
            logging.info("Message number: %s" % (data[2]))
            logging.info("Number of satellites in view: %s" % (data[3]))
            return
            for i in range(4, 17, 4):
                if data[i] != "":
                    logging.info(
                        "Satellite ID: %s Elevation: %s Azimuth: %s SNR: %s"
                        % (data[i], data[i + 1], data[i + 2], data[i + 3])
                    )
            logging.info("Checksum: %s" % (data[20]))

        # Geographic Position - Latitude/Longitude
        elif messageID == "GLL":
            logging.info("Latitude: %s" % (data[1]))
            logging.info("Longitude: %s" % (data[3]))
            logging.info("Time: %s" % (data[5]))
            GLLstatus = ["active", "inactive", "invalid"]
            if   data[6] == "A": _sGLLstatus = GLLstatus[0]
            elif data[6] == "V": _sGLLstatus = GLLstatus[1]
            else:                _sGLLstatus = GLLstatus[2]
            logging.info("Status: %s" % (_sGLLstatus))
            logging.info("Checksum: %s" % (data[7]))

        # GPGGA - Global Positioning System Fix Data
        elif messageID == "GGA":
            logging.info("Time: %s" % (data[1]))
            logging.info("Latitude: %s%s" % (data[2], data[3]))
            logging.info("Longitude: %s%s" % (data[4], data[5]))
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
            logging.info("Fix Quality: %s" % (data[6]))
            logging.info("Fix Quality Details: %s" % (fixDetails[int(data[6])]))
            logging.info("Number of Satellites: %s" % (data[7]))
            HDOP = data[8]  # Horizontal Dilution of Precision = accuracy of horizontal position
            logging.info("Horizontal Dilution of Precision: %s" % (HDOP))
            if HDOP == "0": logging.info("No HDOP available")
            if HDOP >= "6": logging.info("HDOP is terrible")
            logging.info("Altitude ASL: %s %s" % (data[9], data[10]))
            logging.info("Height of Geoid: %s %s" % (data[11], data[12]))
            logging.info("Time since last DGPS update: %s" % (data[13]))
            logging.info("DGPS reference station id: %s" % (data[14]))
            logging.info("Checksum: %s" % (data[15]))

        # GPS Overall Satellite Data
        elif messageID == "GSA":
            mode = ["Automatic", "Manual", "Invalid"]
            if   data[1] == "A": _smode = mode[0]
            elif data[1] == "M": _smode = mode[1]
            else:                _smode = mode[2]
            logging.info("Mode: %s" % _smode)
            type = ["No Fix", "2D", "3D"]
            logging.info("Fix Type: %s" % (type[int(data[2]) - 1]))
            for i in range(12):
                if data[3 + i] != "": logging.info("Satellite ID: %s" % (data[3 + i]))
            logging.info("DOP: %sm" % (data[15]))
            logging.info("HDOP: %sm" % (data[16]))
            logging.info("VDOP: %sm" % (data[17]))
            logging.info("Checksum: %s" % (data[18]))

        # Track Made Good and Ground Speed
        elif messageID == "VTG":
            logging.info("True Track Made Good: %s" % (data[1]))
            logging.info("True Track indicator: %s" % (data[2]))
            logging.info("Magnetic Track Made Good: %s" % (data[3]))
            logging.info("Magnetic Track indicator: %s" % (data[4]))
            logging.info("Ground Speed (knots): %s" % (data[5]))
            logging.info("Ground Speed (km/h): %s" % (data[7]))
            modeIndicator = [
                "Autonomous",
                "Differential",
                "Estimated",
                "Manual",
                "Data not valid",
            ]
            if   data[9] == "A": _smodeIndicator = modeIndicator[0]
            elif data[9] == "D": _smodeIndicator = modeIndicator[1]
            elif data[9] == "E": _smodeIndicator = modeIndicator[2]
            elif data[9] == "M": _smodeIndicator = modeIndicator[3]
            else:                _smodeIndicator = modeIndicator[4]
            logging.info("Mode indicator: %s" % (_smodeIndicator))
            logging.info("Checksum: %s" % (data[10]))

        # abort if unknown header
        else:
            logging.error("Unknown message ID")
            return

        # check checksum
        if self.checkChecksum(receivedData[1:]):  # remove $ from cropped data, because checkChecksum function does not need it
            logging.info("Checksum OK")
        else:
            logging.info("Checksum Invalid or Unavailable")


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
