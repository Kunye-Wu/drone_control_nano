#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from px4_msgs.msg import DistanceSensor

#CHANGE AS NEEDED FOR THE PI
ser = serial.Serial('/dev/ttyUSB0', timeout = None, baudrate=115200, bytesize=serial.EIGHTBITS)

class Distance(Node):
    def __init__(self):
        super().__init__("tera_node")
        self.create_timer(0.01, self.timer_callback)
        self.distance_sensor_pub = self.create_publisher(DistanceSensor, "/fmu/in/distance_sensor", 10)
    def timer_callback(self):
        msg = DistanceSensor()
        ser.write(b'T')
        line = ser.readline()
        line = str(line)[2:-5]
        if "\\" not in line and line != "": # it spits out empty stuff and bytes initially so we just throw that away
            line = float(line)
            self.get_logger().info(str(line))
            msg.min_distance = 200.0
            msg.max_distance = 14000.0
            msg.type = 2
            msg.orientation = 25
            if line == -1: #when it is too close, it spits out -1 sometimes
                msg.current_distance = 200.0
            else:
                msg.current_distance = line
            msg.current_distance /= 1000.0
            self.distance_sensor_pub.publish(msg)

def main(args=None):
    rclpy.init(args = args)
    node = Distance()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
