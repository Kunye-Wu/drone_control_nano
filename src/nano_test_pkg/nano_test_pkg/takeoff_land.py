#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint
from keyboard_msgs.msg import Key

# Note: This is just a simple implementation of a node that arms the simulation drone (no timers, no switching between arming/disarming, etc)

class TakeoffLand(Node):
    def __init__(self):
        super().__init__("takeoff_land")
        self.vc_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.key_subscriber_ = self.create_subscription(Key, "/keydown", self.main_func, 10)
        self.counter = 0
        self.armed = 0.0
        self.alt = 1
        self.timer_ = self.create_timer(0.01, self.main_func)
    def main_func(self, msg: Key):
        if msg.code == 97:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1= not self.armed)
            self.armed = not self.armed
            self.get_logger().info(str(self.armed))
        elif msg.code == 115:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=0.0, param7 = self.alt)
            self.get_logger().info("takeoff")
        elif msg.code == 100:
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_PRECLAND)
            self.get_logger().info("precision landing")
        elif msg.code == 105:
            self.alt += 0.5
            self.get_logger().info("altitude increased: " + str(self.alt))
        elif msg.code == 107:
            self.alt -= 0.5
            self.get_logger().info("altitude decreased: " + str(self.alt))
            
        
    def send_vehicle_command(self, command, param1, param2, param7):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vc_publisher_.publish(msg)
        self.get_logger().info(str(command) + " published to")

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLand()
    rclpy.spin(node)
    rclpy.shutdown()
